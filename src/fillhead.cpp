/**
 * @file fillhead.cpp
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Implements the master controller for the Fillhead system.
 *
 * @details This file provides the concrete implementation for the `Fillhead` class
 * as defined in `fillhead.h`. It contains the logic for the main application loop,
 * command dispatch, state management, and the orchestration of all specialized
 * sub-controllers. The program's entry point, `main()`, is also located here.
 */

//==================================================================================================
// --- Includes ---
//==================================================================================================
#include "fillhead.h"
#include "config.h"
#include "events.h"
#include "commands.h"
#include "error_log.h"
#include "NvmManager.h"
#include <cstring>

using ClearCore::NvmManager;
#include <cstdlib>
#include <cstdio>
#include <sam.h>

//==================================================================================================
// --- Global Watchdog State (Survives Reset) ---
//==================================================================================================
#if WATCHDOG_ENABLED
// Place in .noinit section so it survives reset but is cleared on power-up.
__attribute__((section(".noinit"))) volatile uint32_t g_watchdogRecoveryFlag;
__attribute__((section(".noinit"))) volatile uint32_t g_watchdogBreadcrumb;
// Captured at boot time for accurate reporting (breadcrumb changes during normal operation).
static uint32_t g_crashTimeBreadcrumb = 0;
#endif

//==================================================================================================
// --- Global Instance & sendMessage ---
//==================================================================================================

Fillhead fillhead;

void sendMessage(const char* msg) {
    bool guiDiscovered = fillhead.m_comms.isGuiDiscovered();
    IpAddress targetIp = guiDiscovered ? fillhead.m_comms.getGuiIp() : IpAddress(0, 0, 0, 0);
    uint16_t targetPort = guiDiscovered ? fillhead.m_comms.getGuiPort() : 0;
    fillhead.m_comms.enqueueTx(msg, targetIp, targetPort);
}

//==================================================================================================
// --- Fillhead Class Implementation ---
//==================================================================================================

// --- Constructor & Destructor ---

/**
 * @brief Constructs the Fillhead master controller.
 * @details This constructor uses a member initializer list to instantiate all the
 * specialized sub-controllers. The CommsController object is created first,
 * as its pointer is passed to the other controllers, enabling them to
 * send status messages and telemetry.
 */
Fillhead::Fillhead() :
    // The CommsController object MUST be constructed first.
    m_comms(),
    // Pass the comms pointer to all sub-controllers that need it.
    m_injector(&MOTOR_INJECTOR_A, &MOTOR_INJECTOR_B, this),
    m_injectorValve("inj_valve", &MOTOR_INJECTION_VALVE, HOME_SENSOR_M2, NVM_SLOT_INJ_VALVE_HOME_ON_BOOT, this),
    m_vacuumValve("vac_valve", &MOTOR_VACUUM_VALVE, HOME_SENSOR_M3, NVM_SLOT_VAC_VALVE_HOME_ON_BOOT, this),
    m_heater(this),
    m_vacuum(this)
{
    // Initialize the main system state.
    m_mainState = STATE_STANDBY;

    // Initialize timers for periodic tasks.
    m_lastTelemetryTime = 0;
    m_lastSensorSampleTime = 0;

    // Initialize state machine recovery & homing members.
    m_resetStartTime = 0;
    m_faultGracePeriodEnd = 0;
    m_homingPending = false;
    m_injValveHomingPending = false;
    m_vacValveHomingPending = false;
    m_homingDelayStart = 0;

    // Initialize safety state.
    m_lightCurtainTripped = false;
}


// --- Public Methods: Setup and Main Loop ---

/**
 * @brief Initializes all hardware and sub-controllers for the entire system.
 * @details This method should be called once at startup. It sequentially calls the
 * setup() method for each component in the correct order.
 */
void Fillhead::setup() {
#if WATCHDOG_ENABLED
    // Capture crash-time breadcrumb from previous boot BEFORE overwriting it.
    g_crashTimeBreadcrumb = g_watchdogBreadcrumb;
    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP;
#endif

    // Configure all motors for step and direction control mode.
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_MOTOR_MODE;
#endif
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Log firmware startup.
    g_errorLog.log(LOG_INFO, "=== FIRMWARE STARTUP ===");
    g_errorLog.logf(LOG_INFO, "Firmware version: %s", FIRMWARE_VERSION);

    // Initialize comms first (can take time for network setup).
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_COMMS;
#endif
    m_comms.setup();

#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_MOTOR;
#endif
    m_injector.setup();
    m_injectorValve.setup();
    m_vacuumValve.setup();

    {
        NvmManager &nvmMgr = NvmManager::Instance();
        int32_t rawTorque = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_PINCH_TORQUE));
        int32_t rawStroke = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_PINCH_STROKE));
        float pinchTorque = (rawTorque > 0) ? (float)rawTorque / 100.0f : PINCH_VALVE_PINCH_TORQUE_PERCENT;
        float pinchStroke = (rawStroke > 0) ? (float)rawStroke / 100.0f : PINCH_VALVE_PINCH_STROKE_MM;
        m_injectorValve.setPinchDefaults(pinchTorque, pinchStroke);
        m_vacuumValve.setPinchDefaults(pinchTorque, pinchStroke);
    }

#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_FORCE;
#endif
    m_forceSensor.setup();
    m_heater.setup();
    m_vacuum.setup();

    setupLightCurtain();

#if WATCHDOG_ENABLED
    // Initialize watchdog AFTER comms setup to avoid timeout during network initialization.
    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_WD_RECOVERY;
    handleWatchdogRecovery();

    g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_WD_INIT;
    initializeWatchdog();

    // Feed watchdog immediately after enabling to give maximum time for startup messages.
    feedWatchdog();
#endif

    if (m_mainState != STATE_RECOVERED) {
        m_comms.reportEvent(STATUS_PREFIX_INFO, "Fillhead system setup complete. All components initialized.");
#if WATCHDOG_ENABLED
        feedWatchdog();
#endif
        g_errorLog.log(LOG_INFO, "Setup complete - normal boot");

        if (m_injector.getHomeOnBoot()) {
            m_homingPending = true;
            m_homingDelayStart = 0;
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Auto-homing enabled. Will initiate after startup stabilization...");
        } else {
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Auto-homing disabled. System ready in standby mode.");
        }

        if (m_injectorValve.getHomeOnBoot()) {
            m_injValveHomingPending = true;
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Injection valve auto-home enabled.");
        }
        if (m_vacuumValve.getHomeOnBoot()) {
            m_vacValveHomingPending = true;
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Vacuum valve auto-home enabled.");
        }
#if WATCHDOG_ENABLED
        feedWatchdog();
#endif
    } else {
        g_errorLog.log(LOG_ERROR, "Setup complete - RECOVERED from watchdog");
    }
}

#if WATCHDOG_ENABLED
static const char* resolveBreadcrumb(uint32_t bc) {
    switch (bc) {
        case WD_BREADCRUMB_SAFETY_CHECK:      return "SAFETY_CHECK";
        case WD_BREADCRUMB_COMMS_UPDATE:      return "COMMS_UPDATE";
        case WD_BREADCRUMB_RX_DEQUEUE:        return "RX_DEQUEUE";
        case WD_BREADCRUMB_UPDATE_STATE:      return "UPDATE_STATE";
        case WD_BREADCRUMB_FORCE_UPDATE:      return "FORCE_UPDATE";
        case WD_BREADCRUMB_MOTOR_UPDATE:      return "MOTOR_UPDATE";
        case WD_BREADCRUMB_TELEMETRY:         return "TELEMETRY";
        case WD_BREADCRUMB_UDP_PROCESS:       return "UDP_PROCESS";
        case WD_BREADCRUMB_USB_PROCESS:       return "USB_PROCESS";
        case WD_BREADCRUMB_TX_QUEUE:          return "TX_QUEUE";
        case WD_BREADCRUMB_UDP_SEND:          return "UDP_SEND";
        case WD_BREADCRUMB_NETWORK_REFRESH:   return "NETWORK_REFRESH";
        case WD_BREADCRUMB_USB_SEND:          return "USB_SEND";
        case WD_BREADCRUMB_USB_RECONNECT:     return "USB_RECONNECT";
        case WD_BREADCRUMB_USB_RECOVERY:      return "USB_RECOVERY";
        case WD_BREADCRUMB_REPORT_EVENT:      return "REPORT_EVENT";
        case WD_BREADCRUMB_ENQUEUE_TX:        return "ENQUEUE_TX";
        case WD_BREADCRUMB_MOTOR_IS_FAULT:    return "MOTOR_IS_FAULT";
        case WD_BREADCRUMB_MOTOR_STATE_SWITCH:return "MOTOR_STATE_SWITCH";
        case WD_BREADCRUMB_PROCESS_TX_QUEUE:  return "PROCESS_TX_QUEUE";
        case WD_BREADCRUMB_TX_QUEUE_DEQUEUE:  return "TX_QUEUE_DEQUEUE";
        case WD_BREADCRUMB_TX_QUEUE_UDP:      return "TX_QUEUE_UDP";
        case WD_BREADCRUMB_TX_QUEUE_USB:      return "TX_QUEUE_USB";
        case WD_BREADCRUMB_DISPATCH_CMD:      return "DISPATCH_CMD";
        case WD_BREADCRUMB_PARSE_CMD:         return "PARSE_CMD";
        case WD_BREADCRUMB_MOTOR_FAULT_REPORT:return "MOTOR_FAULT_REPORT";
        case WD_BREADCRUMB_STATE_BUSY_CHECK:  return "STATE_BUSY_CHECK";
        case WD_BREADCRUMB_UDP_PACKET_READ:   return "UDP_PACKET_READ";
        case WD_BREADCRUMB_RX_ENQUEUE:        return "RX_ENQUEUE";
        case WD_BREADCRUMB_USB_AVAILABLE:     return "USB_AVAILABLE";
        case WD_BREADCRUMB_USB_READ:          return "USB_READ";
        case WD_BREADCRUMB_NETWORK_INPUT:     return "NETWORK_INPUT";
        case WD_BREADCRUMB_LWIP_INPUT:        return "LWIP_INPUT";
        case WD_BREADCRUMB_LWIP_TIMEOUT:      return "LWIP_TIMEOUT";
        case WD_BREADCRUMB_SETUP:             return "SETUP";
        case WD_BREADCRUMB_SETUP_MOTOR_MODE:  return "SETUP_MOTOR_MODE";
        case WD_BREADCRUMB_SETUP_COMMS:       return "SETUP_COMMS";
        case WD_BREADCRUMB_SETUP_MOTOR:       return "SETUP_MOTOR";
        case WD_BREADCRUMB_SETUP_FORCE:       return "SETUP_FORCE";
        case WD_BREADCRUMB_SETUP_WD_RECOVERY: return "SETUP_WD_RECOVERY";
        case WD_BREADCRUMB_SETUP_WD_INIT:     return "SETUP_WD_INIT";
        case WD_BREADCRUMB_SETUP_USB:         return "SETUP_USB";
        case WD_BREADCRUMB_SETUP_ETHERNET:    return "SETUP_ETHERNET";
        case WD_BREADCRUMB_SETUP_DHCP:        return "SETUP_DHCP";
        case WD_BREADCRUMB_SETUP_LINK_WAIT:   return "SETUP_LINK_WAIT";
        default:                              return "UNKNOWN";
    }
}
#endif

/**
 * @brief The main execution loop for the Fillhead system.
 * @details This function is called continuously from main(). It orchestrates all
 * real-time operations in a non-blocking manner:
 * 1. Processes communication queues.
 * 2. Dequeues and handles one command per loop.
 * 3. Updates the state machines of all active components.
 * 4. Manages periodic tasks like sensor polling and telemetry transmission.
 */
void Fillhead::loop() {
    // 1. Perform safety checks (including watchdog feed when enabled).
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_SAFETY_CHECK;
#endif
    performSafetyCheck();

    // 2. Process all incoming/outgoing communication queues.
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_COMMS_UPDATE;
#endif
    m_comms.update();

    // 3. Check for and handle one new command from the receive queue.
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_RX_DEQUEUE;
#endif
    Message msg;
    if (m_comms.dequeueRx(msg)) {
        dispatchCommand(msg);
    }

    // 4. Update force sensor readings (every loop iteration for responsiveness).
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_FORCE_UPDATE;
#endif
    m_forceSensor.update();

    // 5. Update the main state machine and all sub-controllers.
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_UPDATE_STATE;
#endif
    updateState();

    // 6. Handle time-based periodic tasks.
    uint32_t now = Milliseconds();
    if (now - m_lastSensorSampleTime >= SENSOR_SAMPLE_INTERVAL_MS) {
        m_lastSensorSampleTime = now;
        m_heater.updateTemperature();
        m_vacuum.updateVacuum();
    }
	
    static uint32_t discoveryTime = 0;
    static bool wasDiscovered = false;
    if (!wasDiscovered && m_comms.isGuiDiscovered()) {
        discoveryTime = now;
        wasDiscovered = true;
    }
    if (!m_comms.isGuiDiscovered()) {
        wasDiscovered = false;
    }
    bool skipForNetworkStability = m_comms.isGuiDiscovered() && (now - discoveryTime < 500);

    if (now - m_lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
#if WATCHDOG_ENABLED
        g_watchdogBreadcrumb = WD_BREADCRUMB_TELEMETRY;
#endif
        m_lastTelemetryTime = now;
        if (!skipForNetworkStability) {
            publishTelemetry();
        }
    }

    if ((m_homingPending || m_injValveHomingPending || m_vacValveHomingPending) && m_mainState == STATE_STANDBY) {
        if (m_homingDelayStart == 0) {
            m_homingDelayStart = now;
        }
        if (m_homingPending && now - m_homingDelayStart > 2000) {
            m_homingPending = false;
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Initiating delayed auto-home...");
            m_injector.handleCommand(CMD_HOME, "");
        }
    }

    if (m_homingDelayStart != 0 && now - m_homingDelayStart > 2000) {
        if (m_injValveHomingPending) {
            m_injValveHomingPending = false;
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Auto-homing injection valve...");
            m_injectorValve.handleCommand(CMD_PINCH_VALVE_HOME, "");
        }
        if (m_vacValveHomingPending) {
            m_vacValveHomingPending = false;
            m_comms.reportEvent(STATUS_PREFIX_INFO, "Auto-homing vacuum valve...");
            m_vacuumValve.handleCommand(CMD_PINCH_VALVE_HOME, "");
        }
    }

    static bool recovery_msg_sent = false;
    if (m_mainState == STATE_RECOVERED && m_comms.isGuiDiscovered() && !recovery_msg_sent) {
        recovery_msg_sent = true;
        char recoveryMsg[128];
        snprintf(recoveryMsg, sizeof(recoveryMsg),
                 "Watchdog timeout at %s (0x%02X). Motors disabled. Send reset to clear.",
                 resolveBreadcrumb(g_crashTimeBreadcrumb),
                 (unsigned int)g_crashTimeBreadcrumb);
        m_comms.reportEvent(STATUS_PREFIX_RECOVERY, recoveryMsg);
    }
    if (m_mainState != STATE_RECOVERED && recovery_msg_sent) {
        recovery_msg_sent = false;
    }
}

void Fillhead::setupLightCurtain() {
#if LIGHT_CURTAIN_ENABLED
    PIN_LIGHT_CURTAIN.Mode(Connector::INPUT_DIGITAL);
    PIN_LIGHT_CURTAIN.FilterLength(LIGHT_CURTAIN_FILTER_MS);
#endif
}

void Fillhead::performSafetyCheck() {
#if LIGHT_CURTAIN_ENABLED
    bool curtainBlocked = (PIN_LIGHT_CURTAIN.State() == LIGHT_CURTAIN_ACTIVE_STATE);
    if (curtainBlocked && !m_lightCurtainTripped) {
        m_lightCurtainTripped = true;
        abort();
        m_mainState = STATE_ERROR;
        reportEvent(STATUS_PREFIX_ERROR, "Light curtain tripped. Motors disabled. Clear obstruction and send reset.");
    }
#endif

#if WATCHDOG_ENABLED
    feedWatchdog();
#endif
}

// --- Private Methods: State, Command, and Telemetry Handling ---

/**
 * @brief Updates the main system state and the state machines of all sub-controllers.
 * @details This function is called once per loop. It first checks for motor faults
 * to determine if the system should be in an error state. If not, it checks if any
 * subsystem is busy. Otherwise, it remains in standby. It then calls the update
 * functions for all sub-controllers.
 */
void Fillhead::updateState() {
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_MOTOR_UPDATE;
#endif
    m_injector.updateState();
    m_injectorValve.updateState();
    m_vacuumValve.updateState();
    m_heater.updateState();
    m_vacuum.updateState();

#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_MOTOR_STATE_SWITCH;
#endif
    switch (m_mainState) {
        case STATE_STANDBY:
        case STATE_BUSY: {
            uint32_t now = Milliseconds();
            bool inGracePeriod = (now < m_faultGracePeriodEnd);

#if WATCHDOG_ENABLED
            g_watchdogBreadcrumb = WD_BREADCRUMB_MOTOR_IS_FAULT;
#endif
            if (!inGracePeriod && (m_injector.isInFault() || m_injectorValve.isInFault() || m_vacuumValve.isInFault())) {
#if WATCHDOG_ENABLED
                g_watchdogBreadcrumb = WD_BREADCRUMB_MOTOR_FAULT_REPORT;
#endif
                m_mainState = STATE_ERROR;
                g_errorLog.log(LOG_ERROR, "Motor fault detected -> ERROR state");
                reportEvent(STATUS_PREFIX_ERROR, "Motor fault detected. System entering ERROR state. Send reset to recover.");
                break;
            }

#if WATCHDOG_ENABLED
            g_watchdogBreadcrumb = WD_BREADCRUMB_STATE_BUSY_CHECK;
#endif
            MainState newState = (m_injector.isBusy() || m_injectorValve.isBusy() || m_vacuumValve.isBusy() || m_vacuum.isBusy())
                ? STATE_BUSY : STATE_STANDBY;
            if (newState != m_mainState) {
                g_errorLog.logf(LOG_DEBUG, "State: %s -> %s",
                    (m_mainState == STATE_STANDBY) ? "STANDBY" : "BUSY",
                    (newState == STATE_STANDBY) ? "STANDBY" : "BUSY");
            }
            m_mainState = newState;
            break;
        }

        case STATE_CLEARING_ERRORS: {
            if (!m_injector.isBusy() && !m_injectorValve.isBusy() && !m_vacuumValve.isBusy() && !m_vacuum.isBusy()) {
                m_injector.disable();
                m_injectorValve.disable();
                m_vacuumValve.disable();
                Delay_ms(10);
                m_injector.enable();
                m_injectorValve.enable();
                m_vacuumValve.enable();

                m_mainState = STATE_STANDBY;
                reportEvent(STATUS_PREFIX_DONE, "CLEAR_ERRORS complete. System is in STANDBY state.");
            }
            break;
        }

        case STATE_RESETTING: {
            uint32_t now = Milliseconds();

            // Phase 1: Wait 100ms after disable before re-enabling
            if (now - m_resetStartTime < 100) break;

            // Phase 2: Start motor enable (only once, when enable state is IDLE)
            if (m_injector.getEnableState() == ENABLE_IDLE) {
                MOTOR_INJECTOR_A.ClearAlerts();
                MOTOR_INJECTOR_B.ClearAlerts();
                MOTOR_VACUUM_VALVE.ClearAlerts();
                MOTOR_INJECTION_VALVE.ClearAlerts();

                m_injector.enable();
                m_injectorValve.enable();
                m_vacuumValve.enable();
                break;
            }

            // Phase 3: Wait for injector enable to complete (non-blocking)
            m_injector.updateEnableState();
            if (!m_injector.isEnableComplete()) break;

            // Phase 4: Enable complete - finish reset
            m_faultGracePeriodEnd = now + 500;
            standby();
            reportEvent(STATUS_PREFIX_DONE, "reset");
            m_mainState = STATE_STANDBY;
            break;
        }

        case STATE_RECOVERED:
            break;

        case STATE_ERROR:
        case STATE_DISABLED:
            break;
    }
}


PinchValve* Fillhead::resolveValve(const char* &args) {
    if (strncmp(args, "injector", 8) == 0) {
        args += 8;
        while (*args == ' ') args++;
        return &m_injectorValve;
    }
    if (strncmp(args, "vacuum", 6) == 0) {
        args += 6;
        while (*args == ' ') args++;
        return &m_vacuumValve;
    }
    return nullptr;
}

/**
 * @brief Master command handler; acts as a switchboard to delegate tasks.
 * @param msg The message object containing the command string and remote IP details.
 * @details This function parses the command from the message buffer and uses a
 * switch statement to delegate it to the appropriate sub-controller or
 * handles system-level commands itself.
 */
void Fillhead::dispatchCommand(const Message& msg) {
#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_PARSE_CMD;
#endif
    Command command_enum = parseCommand(msg.buffer);
    const char* args = getCommandParams(msg.buffer, command_enum);

#if WATCHDOG_ENABLED
    g_watchdogBreadcrumb = WD_BREADCRUMB_DISPATCH_CMD;
#endif

    if (command_enum != CMD_DISCOVER_DEVICE) {
        g_errorLog.logf(LOG_DEBUG, "Dispatch cmd: %s", msg.buffer);
    }

    if (m_mainState == STATE_RECOVERED) {
        if (command_enum != CMD_DISCOVER_DEVICE && command_enum != CMD_RESET && command_enum != CMD_DUMP_ERROR_LOG) {
            m_comms.reportEvent(STATUS_PREFIX_ERROR, "Command ignored: System in RECOVERED state. Send reset to clear.");
            g_errorLog.logf(LOG_WARNING, "Cmd blocked (RECOVERED): %s", msg.buffer);
            return;
        }
    }

    if (m_mainState == STATE_ERROR) {
        if (command_enum != CMD_DISCOVER_DEVICE && command_enum != CMD_RESET && command_enum != CMD_DUMP_ERROR_LOG) {
            m_comms.reportEvent(STATUS_PREFIX_ERROR, "Command ignored: System in ERROR state. Send reset to recover.");
            g_errorLog.logf(LOG_WARNING, "Cmd blocked (ERROR): %s", msg.buffer);
            return;
        }
    }

    if (command_enum == CMD_INJECT) {
        if (!m_injectorValve.isHomed() || !m_injectorValve.isOpen()) {
            reportEvent(STATUS_PREFIX_ERROR, "Injection command ignored: Injection valve must be homed and open.");
            return;
        }
    }

    switch (command_enum) {
        // --- System-Level Commands ---
        case CMD_DISCOVER_DEVICE: {
            char* portStr = strstr(msg.buffer, "PORT=");
            if (portStr) {
                uint16_t guiPort = atoi(portStr + 5);

                IpAddress localhost(127, 0, 0, 1);
                bool fromUsb = (msg.remoteIp == localhost);

                if (!fromUsb) {
                    m_comms.setGuiIp(msg.remoteIp);
                    m_comms.setGuiPort(guiPort);
                    m_comms.setGuiDiscovered(true);
                }

                char discoveryMsg[128];
                snprintf(discoveryMsg, sizeof(discoveryMsg), "%sDEVICE_ID=fillhead PORT=%d FW=%s",
                         STATUS_PREFIX_DISCOVERY, LOCAL_PORT, FIRMWARE_VERSION);
                m_comms.enqueueTx(discoveryMsg, msg.remoteIp, guiPort);
            }
            break;
        }
        case CMD_RESET:         clearErrors(); break;
        case CMD_ENABLE:        enable(); break;
        case CMD_DISABLE:       disable(); break;
        case CMD_ABORT:         abort(); break;

        // --- Pressboi Motion Commands (Delegated to Injector) ---
        case CMD_HOME:
        case CMD_MOVE_ABS:
        case CMD_MOVE_INC:
        case CMD_SET_RETRACT:
        case CMD_RETRACT:
        case CMD_PAUSE:
        case CMD_RESUME:
            m_injector.handleCommand(command_enum, args);
            break;
        case CMD_CANCEL:
            m_injector.handleCommand(command_enum, args);
            abort();
            break;

        // --- Fillhead Injection Commands (Delegated to Injector) ---
        case CMD_MACHINE_HOME:
        case CMD_CARTRIDGE_HOME:
        case CMD_MOVE_TO_CARTRIDGE_HOME:
        case CMD_MOVE_TO_CARTRIDGE_RETRACT:
        case CMD_INJECT:
        case CMD_SET_CARTRIDGE_ML_PER_MM:
        case CMD_PAUSE_INJECTION:
        case CMD_RESUME_INJECTION:
        case CMD_CANCEL_INJECTION:
            m_injector.handleCommand(command_enum, args);
            break;

        // --- Pinch Valve Commands (unified) ---
        case CMD_PINCH_VALVE_HOME:
        case CMD_PINCH_VALVE_OPEN:
        case CMD_PINCH_VALVE_CLOSE:
        case CMD_PINCH_VALVE_JOG: {
            PinchValve* valve = resolveValve(args);
            if (valve) {
                valve->handleCommand(command_enum, args);
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid valve selector. Use 'vacuum' or 'injector'.");
            }
            break;
        }

        case CMD_PINCH_VALVE_HOME_ON_BOOT: {
            PinchValve* valve = resolveValve(args);
            if (!valve) {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid valve selector. Use 'vacuum' or 'injector'.");
                break;
            }
            char enabled[32] = "";
            if (sscanf(args, "%31s", enabled) == 1) {
                if (valve->setHomeOnBoot(enabled)) {
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Pinch valve home on boot set to '%s' and saved to NVM", enabled);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                    reportEvent(STATUS_PREFIX_DONE, "pinch_valve_home_on_boot");
                } else {
                    reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter. Use 'true' or 'false'.");
                }
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for pinch_valve_home_on_boot.");
            }
            break;
        }

        case CMD_SET_VALVE_PINCH_TORQUE: {
            float val = 0;
            if (args && sscanf(args, "%f", &val) == 1 && val > 0.0f && val <= 100.0f) {
                NvmManager &nvmMgr = NvmManager::Instance();
                nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_PINCH_TORQUE), (int32_t)(val * 100.0f));
                m_injectorValve.setPinchDefaults(val, -1);
                m_vacuumValve.setPinchDefaults(val, -1);
                char msg_buf[128];
                snprintf(msg_buf, sizeof(msg_buf), "Valve pinch torque set to %.1f%% and saved to NVM", val);
                reportEvent(STATUS_PREFIX_INFO, msg_buf);
                reportEvent(STATUS_PREFIX_DONE, "set_valve_pinch_torque");
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter. Provide torque percent (0-100).");
            }
            break;
        }

        case CMD_SET_VALVE_PINCH_STROKE: {
            float val = 0;
            if (args && sscanf(args, "%f", &val) == 1 && val > 0.0f && val <= 500.0f) {
                NvmManager &nvmMgr = NvmManager::Instance();
                nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_PINCH_STROKE), (int32_t)(val * 100.0f));
                m_injectorValve.setPinchDefaults(-1, val);
                m_vacuumValve.setPinchDefaults(-1, val);
                char msg_buf[128];
                snprintf(msg_buf, sizeof(msg_buf), "Valve pinch stroke set to %.1fmm and saved to NVM", val);
                reportEvent(STATUS_PREFIX_INFO, msg_buf);
                reportEvent(STATUS_PREFIX_DONE, "set_valve_pinch_stroke");
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter. Provide stroke in mm (0-500).");
            }
            break;
        }

        // --- Heater Commands ---
        case CMD_HEATER_ON:
        case CMD_HEATER_OFF:
        case CMD_SET_HEATER_GAINS:
        case CMD_SET_HEATER_SETPOINT:
            m_heater.handleCommand(command_enum, args);
            break;

        // --- Vacuum Commands ---
        case CMD_VACUUM_ON:
        case CMD_VACUUM_OFF:
        case CMD_VACUUM_LEAK_TEST:
        case CMD_SET_VACUUM_TARGET:
        case CMD_SET_VACUUM_TIMEOUT_S:
        case CMD_SET_LEAK_TEST_DELTA:
        case CMD_SET_LEAK_TEST_DURATION_S:
            m_vacuum.handleCommand(command_enum, args);
            break;

        // --- Pressboi Calibration/Config Commands (handled at system level) ---
        case CMD_SET_FORCE_MODE: {
            char mode[32] = "";
            if (sscanf(args, "%31s", mode) == 1) {
                if (m_injector.setForceMode(mode)) {
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Force mode set to '%s' and saved to NVM", mode);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                    reportEvent(STATUS_PREFIX_DONE, "set_force_mode");
                } else {
                    reportEvent(STATUS_PREFIX_ERROR, "Invalid mode. Use 'motor_torque' or 'load_cell'.");
                }
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for set_force_mode.");
            }
            break;
        }
        case CMD_SET_FORCE_OFFSET: {
            float offset = 0.0f;
            if (sscanf(args, "%f", &offset) == 1) {
                const char* mode = m_injector.getForceMode();
                if (strcmp(mode, "load_cell") == 0) {
                    m_forceSensor.setOffset(offset);
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Load cell offset set to %.2f kg and saved to NVM", offset);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                } else {
                    m_injector.setForceCalibrationOffset(offset);
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Motor torque offset set to %.4f and saved to NVM", offset);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                }
                reportEvent(STATUS_PREFIX_DONE, "set_force_offset");
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for set_force_offset.");
            }
            break;
        }
        case CMD_SET_FORCE_SCALE: {
            float scale = 1.0f;
            if (sscanf(args, "%f", &scale) == 1) {
                const char* mode = m_injector.getForceMode();
                if (strcmp(mode, "load_cell") == 0) {
                    m_forceSensor.setScale(scale);
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Load cell scale set to %.6f and saved to NVM", scale);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                } else {
                    m_injector.setForceCalibrationScale(scale);
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Motor torque scale set to %.6f and saved to NVM", scale);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                }
                reportEvent(STATUS_PREFIX_DONE, "set_force_scale");
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for set_force_scale.");
            }
            break;
        }
        case CMD_SET_FORCE_ZERO: {
            const char* mode = m_injector.getForceMode();
            if (strcmp(mode, "load_cell") == 0) {
                float old_offset = m_forceSensor.getOffset();
                float current_force = m_forceSensor.getForce();
                float new_offset = old_offset - current_force;

                m_forceSensor.setOffset(new_offset);

                char msg_buf[128];
                snprintf(msg_buf, sizeof(msg_buf), "Load cell offset: %.2f kg -> %.2f kg",
                         old_offset, new_offset);
                reportEvent(STATUS_PREFIX_INFO, msg_buf);
            } else {
                float old_offset = m_injector.getForceCalibrationOffset();
                float current_torque = (MOTOR_INJECTOR_A.HlfbPercent() + MOTOR_INJECTOR_B.HlfbPercent()) / 2.0f;
                float new_offset = -current_torque;

                m_injector.setForceCalibrationOffset(new_offset);

                char msg_buf[128];
                snprintf(msg_buf, sizeof(msg_buf), "Motor torque offset: %.4f%% -> %.4f%%",
                         old_offset, new_offset);
                reportEvent(STATUS_PREFIX_INFO, msg_buf);
            }
            reportEvent(STATUS_PREFIX_DONE, "set_force_zero");
            break;
        }
        case CMD_SET_STRAIN_CAL: {
            float x4 = MACHINE_STRAIN_COEFF_X4, x3 = MACHINE_STRAIN_COEFF_X3;
            float x2 = MACHINE_STRAIN_COEFF_X2, x1 = MACHINE_STRAIN_COEFF_X1;
            float c = MACHINE_STRAIN_COEFF_C;
            if (sscanf(args, "%f %f %f %f %f", &x4, &x3, &x2, &x1, &c) == 5) {
                m_injector.setMachineStrainCoeffs(x4, x3, x2, x1, c);
                char msg_buf[160];
                snprintf(msg_buf, sizeof(msg_buf),
                         "Machine strain polynomial updated: f(x) = %.3f x^4 %+.3f x^3 %+.3f x^2 %+.3f x %+.3f",
                         x4, x3, x2, x1, c);
                reportEvent(STATUS_PREFIX_INFO, msg_buf);
                reportEvent(STATUS_PREFIX_DONE, "set_strain_cal");
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameters for set_strain_cal. Need 5 coefficients.");
            }
            break;
        }
        case CMD_SET_POLARITY: {
            char polarity[32] = "";
            if (sscanf(args, "%31s", polarity) == 1) {
                if (m_injector.setPolarity(polarity)) {
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Coordinate system polarity set to '%s' and saved to NVM", polarity);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                    reportEvent(STATUS_PREFIX_DONE, "set_polarity");
                } else {
                    reportEvent(STATUS_PREFIX_ERROR, "Invalid polarity. Use 'normal' or 'inverted'.");
                }
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for set_polarity.");
            }
            break;
        }
        case CMD_HOME_ON_BOOT: {
            char enabled[32] = "";
            if (sscanf(args, "%31s", enabled) == 1) {
                if (m_injector.setHomeOnBoot(enabled)) {
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Home on boot set to '%s' and saved to NVM", enabled);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                    reportEvent(STATUS_PREFIX_DONE, "home_on_boot");
                } else {
                    reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter. Use 'true' or 'false'.");
                }
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for home_on_boot.");
            }
            break;
        }
        case CMD_SET_PRESS_THRESHOLD: {
            float threshold_kg = 0.0f;
            if (sscanf(args, "%f", &threshold_kg) == 1) {
                if (m_injector.setPressThreshold(threshold_kg)) {
                    char msg_buf[128];
                    snprintf(msg_buf, sizeof(msg_buf), "Press threshold set to %.2f kg and saved to NVM", threshold_kg);
                    reportEvent(STATUS_PREFIX_INFO, msg_buf);
                    reportEvent(STATUS_PREFIX_DONE, "set_press_threshold");
                } else {
                    reportEvent(STATUS_PREFIX_ERROR, "Invalid threshold. Must be between 0.1 and 50.0 kg.");
                }
            } else {
                reportEvent(STATUS_PREFIX_ERROR, "Invalid parameter for set_press_threshold.");
            }
            break;
        }

        case CMD_DUMP_ERROR_LOG: {
            int count = g_errorLog.getEntryCount();
            char line[256];
            snprintf(line, sizeof(line), "=== ERROR LOG: %d entries ===", count);
            reportEvent(STATUS_PREFIX_INFO, line);

            const char* levelNames[] = {"DEBUG", "INFO", "WARN", "ERROR", "CRIT"};
            for (int i = 0; i < count; i++) {
                LogEntry entry;
                if (g_errorLog.getEntry(i, &entry)) {
                    const char* lvl = ((int)entry.level >= 0 && (int)entry.level <= 4)
                        ? levelNames[(int)entry.level] : "???";
                    snprintf(line, sizeof(line), "[%lu] %s: %s",
                             (unsigned long)entry.timestamp, lvl, entry.message);
                    reportEvent(STATUS_PREFIX_INFO, line);
                    Delay_ms(5);
                    if ((i + 1) % 5 == 0) {
#if WATCHDOG_ENABLED
                        feedWatchdog();
#endif
                    }
                }
            }
#if WATCHDOG_ENABLED
            feedWatchdog();
#endif
            reportEvent(STATUS_PREFIX_INFO, "=== END ERROR LOG ===");

            int hbCount = g_heartbeatLog.getEntryCount();
            if (hbCount > 0) {
                HeartbeatEntry firstEntry, lastEntry;
                g_heartbeatLog.getEntry(0, &firstEntry);
                g_heartbeatLog.getEntry(hbCount - 1, &lastEntry);
                uint32_t spanMs = lastEntry.timestamp - firstEntry.timestamp;
                uint32_t spanHours = spanMs / 3600000;
                uint32_t spanMins = (spanMs % 3600000) / 60000;
                snprintf(line, sizeof(line), "=== HEARTBEAT LOG: %d entries (%luh%lum span) ===",
                         hbCount, (unsigned long)spanHours, (unsigned long)spanMins);
            } else {
                snprintf(line, sizeof(line), "=== HEARTBEAT LOG: 0 entries ===");
            }
            reportEvent(STATUS_PREFIX_INFO, line);

            for (int i = 0; i < hbCount; i++) {
                HeartbeatEntry hb;
                if (g_heartbeatLog.getEntry(i, &hb)) {
                    snprintf(line, sizeof(line), "[%lu] U:%d N:%d A:%d",
                             (unsigned long)hb.timestamp, hb.usbConnected,
                             hb.networkActive, hb.usbAvailable);
                    reportEvent(STATUS_PREFIX_INFO, line);
                    if ((i + 1) % 10 == 0) {
                        Delay_ms(50);
#if WATCHDOG_ENABLED
                        feedWatchdog();
#endif
                    }
                }
            }
#if WATCHDOG_ENABLED
            feedWatchdog();
#endif
            reportEvent(STATUS_PREFIX_INFO, "=== END HEARTBEAT LOG ===");
            reportEvent(STATUS_PREFIX_DONE, "dump_error_log");
            break;
        }

        case CMD_TEST_WATCHDOG:
            reportEvent(STATUS_PREFIX_INFO, "Triggering watchdog test - system will reset...");
            while (true) {}
            break;

        case CMD_REBOOT_BOOTLOADER: {
#if WATCHDOG_ENABLED
            WDT->CTRLA.reg = 0;
            while(WDT->SYNCBUSY.reg);
#endif
            reportEvent(STATUS_PREFIX_INFO, "Rebooting to bootloader...");
            SysMgr.ResetBoard(SysManager::RESET_TO_BOOTLOADER);
            break;
        }

        case CMD_DUMP_NVM: {
            NvmManager &nvmMgr = NvmManager::Instance();
            char nvm_buf[256];
            int32_t nvm_values[16];
            for (int i = 0; i < 16; ++i) {
                nvm_values[i] = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(i * 4));
            }
            for (int i = 0; i < 16; ++i) {
                int byte_offset = i * 4;
                int32_t value = nvm_values[i];
                unsigned char* bytes = (unsigned char*)&value;
                char hex_str[50];
                char ascii_str[10];
                snprintf(hex_str, sizeof(hex_str), "%02X %02X %02X %02X",
                         bytes[0], bytes[1], bytes[2], bytes[3]);
                for (int j = 0; j < 4; j++) {
                    ascii_str[j] = (bytes[j] >= 32 && bytes[j] <= 126) ? bytes[j] : '.';
                }
                ascii_str[4] = '\0';
                snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:%04X:%s:%s", byte_offset, hex_str, ascii_str);
                sendMessage(nvm_buf);
            }
            int32_t magic = nvm_values[7];
            const char* magic_status = (magic == 0x464C4831) ? "OK" : "INVALID";
            const char* mode_str = (nvm_values[4] == 0) ? "motor_torque" : "load_cell";
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: Magic=0x%08X(%s) Mode=%s",
                     (unsigned int)magic, magic_status, mode_str);
            sendMessage(nvm_buf);

            // Motor torque calibration (NVM slots 5 & 6 - fixed-point)
            float mt_scale = (float)nvm_values[5] / 100000.0f;
            float mt_offset = (float)nvm_values[6] / 10000.0f;
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: MotorTorque: Scale=%.6f Offset=%.4f %%",
                     mt_scale, mt_offset);
            sendMessage(nvm_buf);

            const char* pol_str = (nvm_values[3] == 1) ? "inverted" : "normal";
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: Polarity=%s HomeOnBoot=%s",
                     pol_str, (nvm_values[13] == 1) ? "true" : "false");
            sendMessage(nvm_buf);
            float retract_mm = 0.0f;
            if (nvm_values[14] != 0 && nvm_values[14] != -1) memcpy(&retract_mm, &nvm_values[14], sizeof(float));
            float threshold_kg = 2.0f;
            if (nvm_values[15] != 0 && nvm_values[15] != -1) memcpy(&threshold_kg, &nvm_values[15], sizeof(float));
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: Retract=%.2fmm PressThreshold=%.2fkg",
                     retract_mm, threshold_kg);
            sendMessage(nvm_buf);

            // Machine strain coefficients (NVM slots 8-12 - IEEE float as bits)
            float strain_coeffs[5];
            const float default_coeffs[5] = {MACHINE_STRAIN_COEFF_X4, MACHINE_STRAIN_COEFF_X3,
                                              MACHINE_STRAIN_COEFF_X2, MACHINE_STRAIN_COEFF_X1,
                                              MACHINE_STRAIN_COEFF_C};
            for (int j = 0; j < 5; j++) {
                int32_t coeff_bits = nvm_values[8 + j];
                if (coeff_bits != 0 && coeff_bits != -1) {
                    memcpy(&strain_coeffs[j], &coeff_bits, sizeof(float));
                } else {
                    strain_coeffs[j] = default_coeffs[j];
                }
            }
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: StrainCoeffs x4=%.4f x3=%.4f x2=%.4f x1=%.4f c=%.4f",
                     strain_coeffs[0], strain_coeffs[1], strain_coeffs[2], strain_coeffs[3], strain_coeffs[4]);
            sendMessage(nvm_buf);

            // Pinch valve home-on-boot flags (NVM slots 16 & 17)
            NvmManager &nvmMgrValve = NvmManager::Instance();
            int32_t injValveHob = nvmMgrValve.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_INJ_VALVE_HOME_ON_BOOT));
            int32_t vacValveHob = nvmMgrValve.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_VAC_VALVE_HOME_ON_BOOT));
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: InjValveHomeOnBoot=%s VacValveHomeOnBoot=%s",
                     (injValveHob == 1) ? "true" : "false",
                     (vacValveHob == 1) ? "true" : "false");
            sendMessage(nvm_buf);

            int32_t rawPinchTorque = nvmMgrValve.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_PINCH_TORQUE));
            int32_t rawPinchStroke = nvmMgrValve.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_PINCH_STROKE));
            float pinchTorque = (rawPinchTorque > 0) ? (float)rawPinchTorque / 100.0f : PINCH_VALVE_PINCH_TORQUE_PERCENT;
            float pinchStroke = (rawPinchStroke > 0) ? (float)rawPinchStroke / 100.0f : PINCH_VALVE_PINCH_STROKE_MM;
            snprintf(nvm_buf, sizeof(nvm_buf), "NVMDUMP:fillhead:SUMMARY: PinchTorque=%.1f%% PinchStroke=%.1fmm",
                     pinchTorque, pinchStroke);
            sendMessage(nvm_buf);

            reportEvent(STATUS_PREFIX_DONE, "dump_nvm");
            break;
        }

        case CMD_RESET_NVM: {
            NvmManager &nvmMgr = NvmManager::Instance();
            for (int i = 0; i <= 20; ++i) {
                nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(i * 4), -1);
            }
            reportEvent(STATUS_PREFIX_INFO, "All NVM locations (slots 0-20) reset to erased state. Reboot required.");
            reportEvent(STATUS_PREFIX_DONE, "reset_nvm");
            break;
        }

        case CMD_UNKNOWN:
        default:
            m_comms.reportEvent(STATUS_PREFIX_ERROR, "Unknown command sent to Fillhead.");
            break;
    }
}

/**
 * @brief Aggregates telemetry data from all sub-controllers and sends it as a single
 *        message via the communications controller.
 *
 * @details The CommsController will route this to the discovered GUI over UDP when
 *          available, and will also mirror it to USB whenever a USB host is connected.
 */
void Fillhead::publishTelemetry() {
    char telemetryBuffer[1024];
    const char* mainStateStr;
    switch(m_mainState) {
        case STATE_STANDBY:        mainStateStr = "STANDBY"; break;
        case STATE_BUSY:           mainStateStr = "BUSY"; break;
        case STATE_ERROR:          mainStateStr = "ERROR"; break;
        case STATE_DISABLED:       mainStateStr = "DISABLED"; break;
        case STATE_CLEARING_ERRORS:mainStateStr = "CLEARING_ERRORS"; break;
        case STATE_RESETTING:      mainStateStr = "RESETTING"; break;
        case STATE_RECOVERED:      mainStateStr = "RECOVERED"; break;
        default:                   mainStateStr = "UNKNOWN"; break;
    }

    snprintf(telemetryBuffer, sizeof(telemetryBuffer),
        "%s"
        "fillhead_state:%s,"
        "%s,"
        "%s,"
        "%s,"
        "%s,"
        "%s,"
        "inj_st:%s,inj_v_st:%s,vac_v_st:%s,h_st_str:%s,vac_st_str:%s,"
        "lc:%d",
        TELEM_PREFIX,
        mainStateStr,
        m_injector.getTelemetryString(),
        m_injectorValve.getTelemetryString(),
        m_vacuumValve.getTelemetryString(),
        m_heater.getTelemetryString(),
        m_vacuum.getTelemetryString(),
        m_injector.getState(),
        m_injectorValve.getState(),
        m_vacuumValve.getState(),
        m_heater.getState(),
        m_vacuum.getState(),
        (int)m_lightCurtainTripped
    );

    bool guiDiscovered = m_comms.isGuiDiscovered();
    IpAddress targetIp = guiDiscovered ? m_comms.getGuiIp() : IpAddress(0, 0, 0, 0);
    uint16_t targetPort = guiDiscovered ? m_comms.getGuiPort() : 0;
    m_comms.enqueueTx(telemetryBuffer, targetIp, targetPort);
}

#if WATCHDOG_ENABLED
/**
 * @brief Handle recovery from a previous watchdog reset.
 */
void Fillhead::handleWatchdogRecovery() {
    uint8_t reset_cause = RSTC->RCAUSE.reg;
    bool is_watchdog_reset = (reset_cause & RSTC_RCAUSE_WDT) != 0;

    if (is_watchdog_reset) {
        m_injector.disable();
        m_injectorValve.disable();
        m_vacuumValve.disable();
        m_mainState = STATE_RECOVERED;

        char debug_msg[120];
        snprintf(debug_msg, sizeof(debug_msg),
                 "Reset cause: 0x%02X (POR=%d BODCORE=%d BODVDD=%d EXT=%d WDT=%d SYST=%d)",
                 reset_cause,
                 (reset_cause & RSTC_RCAUSE_POR) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_BODCORE) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_BODVDD) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_EXT) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_WDT) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_SYST) ? 1 : 0);
        m_comms.reportEvent(STATUS_PREFIX_INFO, debug_msg);

        MOTOR_INJECTOR_A.ClearAlerts();
        MOTOR_INJECTOR_B.ClearAlerts();
        MOTOR_VACUUM_VALVE.ClearAlerts();
        MOTOR_INJECTION_VALVE.ClearAlerts();

        char recoveryMsg[128];
        snprintf(recoveryMsg, sizeof(recoveryMsg),
                 "Watchdog timeout in %s - main loop blocked >256ms. Motors disabled. Send RESET to clear.",
                 resolveBreadcrumb(g_crashTimeBreadcrumb));
        m_comms.reportEvent(STATUS_PREFIX_RECOVERY, recoveryMsg);

        ConnectorLed.Mode(Connector::OUTPUT_DIGITAL);
        ConnectorLed.State(true);
    } else {
        char debug_msg[120];
        snprintf(debug_msg, sizeof(debug_msg),
                 "Reset cause: 0x%02X (POR=%d BODCORE=%d BODVDD=%d EXT=%d WDT=%d SYST=%d)",
                 reset_cause,
                 (reset_cause & RSTC_RCAUSE_POR) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_BODCORE) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_BODVDD) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_EXT) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_WDT) ? 1 : 0,
                 (reset_cause & RSTC_RCAUSE_SYST) ? 1 : 0);
        m_comms.reportEvent(STATUS_PREFIX_INFO, debug_msg);
    }
}

/**
 * @brief Initialize the hardware watchdog timer.
 */
void Fillhead::initializeWatchdog() {
    // Disable watchdog during configuration
    WDT->CTRLA.reg = 0;
    while(WDT->SYNCBUSY.reg);

    // Period value 0x5 = 256 cycles at 1kHz WDT clock ≈ 256ms
    uint8_t per_value = 0x5;

    WDT->CONFIG.reg = WDT_CONFIG_PER(per_value);
    while(WDT->SYNCBUSY.reg);

    // Enable early warning interrupt
    WDT->INTENSET.reg = WDT_INTENSET_EW;

    NVIC_EnableIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0);  // Highest priority

    // Enable watchdog
    WDT->CTRLA.reg = WDT_CTRLA_ENABLE;
    while(WDT->SYNCBUSY.reg);

    g_errorLog.log(LOG_INFO, "Watchdog initialized (256ms timeout)");
}

/**
 * @brief Feed (reset) the watchdog timer.
 */
void Fillhead::feedWatchdog() {
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    while (WDT->SYNCBUSY.reg) {
    }
}

void Fillhead::clearWatchdogRecovery() {
    if (m_mainState == STATE_RECOVERED) {
        reportEvent(STATUS_PREFIX_INFO, "Clearing watchdog recovery state...");
        ConnectorLed.State(false);
    }
}

/**
 * @brief Watchdog early-warning interrupt handler for Fillhead.
 */
extern "C" void WDT_Handler(void) {
    // Clear the early warning interrupt flag.
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;

    // Immediately disable all motors to prevent damage.
    MOTOR_INJECTOR_A.EnableRequest(false);
    MOTOR_INJECTOR_B.EnableRequest(false);
    MOTOR_VACUUM_VALVE.EnableRequest(false);
    MOTOR_INJECTION_VALVE.EnableRequest(false);

    // Indicate watchdog trigger on the ClearCore status LED.
    ConnectorLed.Mode(Connector::OUTPUT_DIGITAL);
    ConnectorLed.State(true);

    // Set recovery flag in .noinit memory (survives reset).
    g_watchdogRecoveryFlag = WATCHDOG_RECOVERY_FLAG;

    // Brief LED blink pattern to indicate watchdog event.
    for (int i = 0; i < 5; i++) {
        ConnectorLed.State(true);
        for (volatile int d = 0; d < 5000; d++);
        ConnectorLed.State(false);
        for (volatile int d = 0; d < 5000; d++);
    }
}
#endif

/**
 * @brief Enables all motors and places the system in a ready state.
 */
void Fillhead::enable() {
    if (m_mainState == STATE_DISABLED) {
        m_mainState = STATE_STANDBY;
        m_injector.enable();
        m_injectorValve.enable();
        m_vacuumValve.enable();
        m_comms.reportEvent(STATUS_PREFIX_DONE, "enable");
    } else {
        m_comms.reportEvent(STATUS_PREFIX_INFO, "System already enabled.");
    }
}

/**
 * @brief Disables all motors and stops all operations.
 */
void Fillhead::disable() {
    abort(); // Safest to abort any motion first.
    m_mainState = STATE_DISABLED;
    m_injector.disable();
    m_injectorValve.disable();
    m_vacuumValve.disable();
    m_comms.reportEvent(STATUS_PREFIX_DONE, "disable");
}

/**
 * @brief Halts all motion and resets the system state to standby.
 */
void Fillhead::abort() {
    reportEvent(STATUS_PREFIX_INFO, "ABORT received. Stopping all motion.");
    m_injector.abortMove();
    m_injectorValve.abort();
    m_vacuumValve.abort();
    standby(); // This now resets all sub-controller states.
    reportEvent(STATUS_PREFIX_DONE, "ABORT complete.");
}

/**
 * @brief Resets any error states, clears motor faults, and returns the system to standby.
 */
void Fillhead::clearErrors() {
#if LIGHT_CURTAIN_ENABLED
    if (PIN_LIGHT_CURTAIN.State() == LIGHT_CURTAIN_ACTIVE_STATE) {
        reportEvent(STATUS_PREFIX_ERROR, "Cannot reset: light curtain still blocked.");
        return;
    }
    m_lightCurtainTripped = false;
#endif

    reportEvent(STATUS_PREFIX_INFO, "Reset received. Clearing errors...");
#if WATCHDOG_ENABLED
    clearWatchdogRecovery();
#endif
    m_injector.abortMove();
    m_injectorValve.abort();
    m_vacuumValve.abort();
    m_injector.disable();
    m_injectorValve.disable();
    m_vacuumValve.disable();
    m_resetStartTime = Milliseconds();
    m_mainState = STATE_RESETTING;
}

void Fillhead::standby() {
    // Reset all sub-controllers to their idle states.
    m_injector.reset();
    m_injectorValve.reset();
    m_vacuumValve.reset();
    m_vacuum.resetState();

    // Set the main state and report.
    m_mainState = STATE_STANDBY;
    reportEvent(STATUS_PREFIX_INFO, "System is in STANDBY state.");
}

/**
 * @brief Public interface to send a status message.
 * @details This wrapper method allows owned objects (like an Injector) to send
 * status messages (INFO, DONE, ERROR) through the Fillhead's CommsController
 * without needing a direct pointer to it.
 * @param statusType The prefix for the message (e.g., "INFO: ").
 * @param message The content of the message to send.
 */
void Fillhead::reportEvent(const char* statusType, const char* message) {
    m_comms.reportEvent(statusType, message);
}


//==================================================================================================
// --- Program Entry Point ---
//==================================================================================================

/**
 * @brief The main function and entry point of the application.
 */
int main(void) {
    // Initialize all hardware and controllers.
    fillhead.setup();

    // Enter the main application loop, which runs forever.
    while (true) {
        fillhead.loop();
    }
}
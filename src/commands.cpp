/**
 * @file commands.cpp
 * @brief Command parsing implementation for the Fillhead controller.
 */

#include "commands.h"
#include <string.h>

Command parseCommand(const char* cmdStr) {
    // IMPORTANT: Check longer commands BEFORE shorter ones that are prefixes!
    
    if (strncmp(cmdStr, CMD_STR_DISCOVER_DEVICE, strlen(CMD_STR_DISCOVER_DEVICE)) == 0) return CMD_DISCOVER_DEVICE;
    if (strncmp(cmdStr, CMD_STR_REBOOT_BOOTLOADER, strlen(CMD_STR_REBOOT_BOOTLOADER)) == 0) return CMD_REBOOT_BOOTLOADER;
    if (strncmp(cmdStr, CMD_STR_TEST_WATCHDOG, strlen(CMD_STR_TEST_WATCHDOG)) == 0) return CMD_TEST_WATCHDOG;
    if (strncmp(cmdStr, CMD_STR_DUMP_ERROR_LOG, strlen(CMD_STR_DUMP_ERROR_LOG)) == 0) return CMD_DUMP_ERROR_LOG;
    if (strncmp(cmdStr, CMD_STR_RESET_NVM, strlen(CMD_STR_RESET_NVM)) == 0) return CMD_RESET_NVM;
    if (strncmp(cmdStr, CMD_STR_DUMP_NVM, strlen(CMD_STR_DUMP_NVM)) == 0) return CMD_DUMP_NVM;

    // Force/calibration (before shorter commands)
    if (strncmp(cmdStr, CMD_STR_SET_FORCE_OFFSET, strlen(CMD_STR_SET_FORCE_OFFSET)) == 0) return CMD_SET_FORCE_OFFSET;
    if (strncmp(cmdStr, CMD_STR_SET_FORCE_ZERO, strlen(CMD_STR_SET_FORCE_ZERO)) == 0) return CMD_SET_FORCE_ZERO;
    if (strncmp(cmdStr, CMD_STR_SET_FORCE_SCALE, strlen(CMD_STR_SET_FORCE_SCALE)) == 0) return CMD_SET_FORCE_SCALE;
    if (strncmp(cmdStr, CMD_STR_SET_FORCE_MODE, strlen(CMD_STR_SET_FORCE_MODE)) == 0) return CMD_SET_FORCE_MODE;
    if (strncmp(cmdStr, CMD_STR_SET_STRAIN_CAL, strlen(CMD_STR_SET_STRAIN_CAL)) == 0) return CMD_SET_STRAIN_CAL;
    if (strncmp(cmdStr, CMD_STR_SET_POLARITY, strlen(CMD_STR_SET_POLARITY)) == 0) return CMD_SET_POLARITY;
    if (strncmp(cmdStr, CMD_STR_SET_PRESS_THRESHOLD, strlen(CMD_STR_SET_PRESS_THRESHOLD)) == 0) return CMD_SET_PRESS_THRESHOLD;
    if (strncmp(cmdStr, CMD_STR_HOME_ON_BOOT, strlen(CMD_STR_HOME_ON_BOOT)) == 0) return CMD_HOME_ON_BOOT;

    // Fillhead injection commands (longer prefixes first)
    if (strncmp(cmdStr, CMD_STR_INJECTION_VALVE_HOME_ON_BOOT, strlen(CMD_STR_INJECTION_VALVE_HOME_ON_BOOT)) == 0) return CMD_INJECTION_VALVE_HOME_ON_BOOT;
    if (strncmp(cmdStr, CMD_STR_INJECTION_VALVE_HOME, strlen(CMD_STR_INJECTION_VALVE_HOME)) == 0) return CMD_INJECTION_VALVE_HOME;
    if (strncmp(cmdStr, CMD_STR_INJECTION_VALVE_OPEN, strlen(CMD_STR_INJECTION_VALVE_OPEN)) == 0) return CMD_INJECTION_VALVE_OPEN;
    if (strncmp(cmdStr, CMD_STR_INJECTION_VALVE_CLOSE, strlen(CMD_STR_INJECTION_VALVE_CLOSE)) == 0) return CMD_INJECTION_VALVE_CLOSE;
    if (strncmp(cmdStr, CMD_STR_INJECTION_VALVE_JOG, strlen(CMD_STR_INJECTION_VALVE_JOG)) == 0) return CMD_INJECTION_VALVE_JOG;

    if (strncmp(cmdStr, CMD_STR_VACUUM_VALVE_HOME_ON_BOOT, strlen(CMD_STR_VACUUM_VALVE_HOME_ON_BOOT)) == 0) return CMD_VACUUM_VALVE_HOME_ON_BOOT;
    if (strncmp(cmdStr, CMD_STR_VACUUM_VALVE_HOME, strlen(CMD_STR_VACUUM_VALVE_HOME)) == 0) return CMD_VACUUM_VALVE_HOME;
    if (strncmp(cmdStr, CMD_STR_VACUUM_VALVE_OPEN, strlen(CMD_STR_VACUUM_VALVE_OPEN)) == 0) return CMD_VACUUM_VALVE_OPEN;
    if (strncmp(cmdStr, CMD_STR_VACUUM_VALVE_CLOSE, strlen(CMD_STR_VACUUM_VALVE_CLOSE)) == 0) return CMD_VACUUM_VALVE_CLOSE;
    if (strncmp(cmdStr, CMD_STR_VACUUM_VALVE_JOG, strlen(CMD_STR_VACUUM_VALVE_JOG)) == 0) return CMD_VACUUM_VALVE_JOG;

    // Heater commands
    if (strncmp(cmdStr, CMD_STR_SET_HEATER_SETPOINT, strlen(CMD_STR_SET_HEATER_SETPOINT)) == 0) return CMD_SET_HEATER_SETPOINT;
    if (strncmp(cmdStr, CMD_STR_SET_HEATER_GAINS, strlen(CMD_STR_SET_HEATER_GAINS)) == 0) return CMD_SET_HEATER_GAINS;
    if (strncmp(cmdStr, CMD_STR_HEATER_ON, strlen(CMD_STR_HEATER_ON)) == 0) return CMD_HEATER_ON;
    if (strncmp(cmdStr, CMD_STR_HEATER_OFF, strlen(CMD_STR_HEATER_OFF)) == 0) return CMD_HEATER_OFF;

    // Vacuum system commands (longer first)
    if (strncmp(cmdStr, CMD_STR_SET_VACUUM_TIMEOUT_S, strlen(CMD_STR_SET_VACUUM_TIMEOUT_S)) == 0) return CMD_SET_VACUUM_TIMEOUT_S;
    if (strncmp(cmdStr, CMD_STR_SET_VACUUM_TARGET, strlen(CMD_STR_SET_VACUUM_TARGET)) == 0) return CMD_SET_VACUUM_TARGET;
    if (strncmp(cmdStr, CMD_STR_SET_LEAK_TEST_DURATION_S, strlen(CMD_STR_SET_LEAK_TEST_DURATION_S)) == 0) return CMD_SET_LEAK_TEST_DURATION_S;
    if (strncmp(cmdStr, CMD_STR_SET_LEAK_TEST_DELTA, strlen(CMD_STR_SET_LEAK_TEST_DELTA)) == 0) return CMD_SET_LEAK_TEST_DELTA;
    if (strncmp(cmdStr, CMD_STR_VACUUM_LEAK_TEST, strlen(CMD_STR_VACUUM_LEAK_TEST)) == 0) return CMD_VACUUM_LEAK_TEST;
    if (strncmp(cmdStr, CMD_STR_VACUUM_ON, strlen(CMD_STR_VACUUM_ON)) == 0) return CMD_VACUUM_ON;
    if (strncmp(cmdStr, CMD_STR_VACUUM_OFF, strlen(CMD_STR_VACUUM_OFF)) == 0) return CMD_VACUUM_OFF;

    // Injection motion commands (longer first)
    if (strncmp(cmdStr, CMD_STR_SET_CARTRIDGE_ML_PER_MM, strlen(CMD_STR_SET_CARTRIDGE_ML_PER_MM)) == 0) return CMD_SET_CARTRIDGE_ML_PER_MM;
    if (strncmp(cmdStr, CMD_STR_INJECT, strlen(CMD_STR_INJECT)) == 0) return CMD_INJECT;
    if (strncmp(cmdStr, CMD_STR_MOVE_TO_CARTRIDGE_RETRACT, strlen(CMD_STR_MOVE_TO_CARTRIDGE_RETRACT)) == 0) return CMD_MOVE_TO_CARTRIDGE_RETRACT;
    if (strncmp(cmdStr, CMD_STR_MOVE_TO_CARTRIDGE_HOME, strlen(CMD_STR_MOVE_TO_CARTRIDGE_HOME)) == 0) return CMD_MOVE_TO_CARTRIDGE_HOME;
    if (strncmp(cmdStr, CMD_STR_CARTRIDGE_HOME_MOVE, strlen(CMD_STR_CARTRIDGE_HOME_MOVE)) == 0) return CMD_CARTRIDGE_HOME_MOVE;
    if (strncmp(cmdStr, CMD_STR_CARTRIDGE_HOME, strlen(CMD_STR_CARTRIDGE_HOME)) == 0) return CMD_CARTRIDGE_HOME;
    if (strncmp(cmdStr, CMD_STR_MACHINE_HOME_MOVE, strlen(CMD_STR_MACHINE_HOME_MOVE)) == 0) return CMD_MACHINE_HOME_MOVE;
    if (strncmp(cmdStr, CMD_STR_MACHINE_HOME, strlen(CMD_STR_MACHINE_HOME)) == 0) return CMD_MACHINE_HOME;
    if (strncmp(cmdStr, CMD_STR_PAUSE_INJECTION, strlen(CMD_STR_PAUSE_INJECTION)) == 0) return CMD_PAUSE_INJECTION;
    if (strncmp(cmdStr, CMD_STR_RESUME_INJECTION, strlen(CMD_STR_RESUME_INJECTION)) == 0) return CMD_RESUME_INJECTION;
    if (strncmp(cmdStr, CMD_STR_CANCEL_INJECTION, strlen(CMD_STR_CANCEL_INJECTION)) == 0) return CMD_CANCEL_INJECTION;
    if (strncmp(cmdStr, CMD_STR_JOG_MOVE, strlen(CMD_STR_JOG_MOVE)) == 0) return CMD_JOG_MOVE;

    // General motion (from Pressboi - shorter, must be AFTER longer prefixes)
    if (strncmp(cmdStr, CMD_STR_SET_RETRACT, strlen(CMD_STR_SET_RETRACT)) == 0) return CMD_SET_RETRACT;
    if (strncmp(cmdStr, CMD_STR_MOVE_ABS, strlen(CMD_STR_MOVE_ABS)) == 0) return CMD_MOVE_ABS;
    if (strncmp(cmdStr, CMD_STR_MOVE_INC, strlen(CMD_STR_MOVE_INC)) == 0) return CMD_MOVE_INC;
    if (strncmp(cmdStr, CMD_STR_RETRACT, strlen(CMD_STR_RETRACT)) == 0) return CMD_RETRACT;
    if (strncmp(cmdStr, CMD_STR_RESET, strlen(CMD_STR_RESET)) == 0) return CMD_RESET;
    if (strncmp(cmdStr, CMD_STR_HOME, strlen(CMD_STR_HOME)) == 0) return CMD_HOME;
    if (strncmp(cmdStr, CMD_STR_PAUSE, strlen(CMD_STR_PAUSE)) == 0) return CMD_PAUSE;
    if (strncmp(cmdStr, CMD_STR_RESUME, strlen(CMD_STR_RESUME)) == 0) return CMD_RESUME;
    if (strncmp(cmdStr, CMD_STR_CANCEL, strlen(CMD_STR_CANCEL)) == 0) return CMD_CANCEL;
    if (strncmp(cmdStr, CMD_STR_ENABLE, strlen(CMD_STR_ENABLE)) == 0) return CMD_ENABLE;
    if (strncmp(cmdStr, CMD_STR_DISABLE, strlen(CMD_STR_DISABLE)) == 0) return CMD_DISABLE;
    if (strncmp(cmdStr, CMD_STR_ABORT, strlen(CMD_STR_ABORT)) == 0) return CMD_ABORT;

    return CMD_UNKNOWN;
}

const char* getCommandParams(const char* cmdStr, Command cmd) {
    switch (cmd) {
        case CMD_MOVE_ABS:            return cmdStr + strlen(CMD_STR_MOVE_ABS);
        case CMD_MOVE_INC:            return cmdStr + strlen(CMD_STR_MOVE_INC);
        case CMD_SET_FORCE_MODE:      return cmdStr + strlen(CMD_STR_SET_FORCE_MODE);
        case CMD_SET_RETRACT:         return cmdStr + strlen(CMD_STR_SET_RETRACT);
        case CMD_RETRACT:             return cmdStr + strlen(CMD_STR_RETRACT);
        case CMD_SET_FORCE_OFFSET:    return cmdStr + strlen(CMD_STR_SET_FORCE_OFFSET);
        case CMD_SET_FORCE_SCALE:     return cmdStr + strlen(CMD_STR_SET_FORCE_SCALE);
        case CMD_SET_STRAIN_CAL:      return cmdStr + strlen(CMD_STR_SET_STRAIN_CAL);
        case CMD_SET_POLARITY:        return cmdStr + strlen(CMD_STR_SET_POLARITY);
        case CMD_HOME_ON_BOOT:        return cmdStr + strlen(CMD_STR_HOME_ON_BOOT);
        case CMD_SET_PRESS_THRESHOLD: return cmdStr + strlen(CMD_STR_SET_PRESS_THRESHOLD);
        case CMD_INJECT:              return cmdStr + strlen(CMD_STR_INJECT);
        case CMD_SET_CARTRIDGE_ML_PER_MM: return cmdStr + strlen(CMD_STR_SET_CARTRIDGE_ML_PER_MM);
        case CMD_JOG_MOVE:            return cmdStr + strlen(CMD_STR_JOG_MOVE);
        case CMD_MOVE_TO_CARTRIDGE_RETRACT: return cmdStr + strlen(CMD_STR_MOVE_TO_CARTRIDGE_RETRACT);
        case CMD_INJECTION_VALVE_JOG: return cmdStr + strlen(CMD_STR_INJECTION_VALVE_JOG);
        case CMD_INJECTION_VALVE_HOME_ON_BOOT: return cmdStr + strlen(CMD_STR_INJECTION_VALVE_HOME_ON_BOOT);
        case CMD_VACUUM_VALVE_JOG:    return cmdStr + strlen(CMD_STR_VACUUM_VALVE_JOG);
        case CMD_VACUUM_VALVE_HOME_ON_BOOT: return cmdStr + strlen(CMD_STR_VACUUM_VALVE_HOME_ON_BOOT);
        case CMD_SET_HEATER_GAINS:    return cmdStr + strlen(CMD_STR_SET_HEATER_GAINS);
        case CMD_SET_HEATER_SETPOINT: return cmdStr + strlen(CMD_STR_SET_HEATER_SETPOINT);
        case CMD_SET_VACUUM_TARGET:   return cmdStr + strlen(CMD_STR_SET_VACUUM_TARGET);
        case CMD_SET_VACUUM_TIMEOUT_S: return cmdStr + strlen(CMD_STR_SET_VACUUM_TIMEOUT_S);
        case CMD_SET_LEAK_TEST_DELTA: return cmdStr + strlen(CMD_STR_SET_LEAK_TEST_DELTA);
        case CMD_SET_LEAK_TEST_DURATION_S: return cmdStr + strlen(CMD_STR_SET_LEAK_TEST_DURATION_S);
        case CMD_VACUUM_ON:               return cmdStr + strlen(CMD_STR_VACUUM_ON);
        case CMD_HEATER_ON:               return cmdStr + strlen(CMD_STR_HEATER_ON);
        case CMD_VACUUM_LEAK_TEST:        return cmdStr + strlen(CMD_STR_VACUUM_LEAK_TEST);
        default:
            return NULL;
    }
}

/**
 * @file comms_controller.cpp
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Implements the CommsController for handling network and serial communications.
 *
 * @details This file provides the concrete implementation for the methods declared in
 * `comms_controller.h`. It includes the logic for managing the circular message queues,
 * processing UDP packets, and parsing command strings.
 */
#include "comms_controller.h"
#include "config.h"
#include "error_log.h"

#if WATCHDOG_ENABLED
extern volatile uint32_t g_watchdogBreadcrumb;
#endif

CommsController::CommsController() {
	m_guiDiscovered = false;
	m_guiPort = 0;
	
	m_rxQueueHead = 0;
	m_rxQueueTail = 0;
	m_txQueueHead = 0;
	m_txQueueTail = 0;
	
	// USB host health tracking - start pessimistic (wait for first sign of host)
	m_lastUsbHealthy = 0;
	m_usbHostConnected = false;
	
	g_errorLog.log(LOG_INFO, "CommsController initialized");
}

void CommsController::setup() {
#if WATCHDOG_ENABLED
	extern volatile uint32_t g_watchdogBreadcrumb;
	g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_USB;
#endif
	setupUsbSerial();

#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_ETHERNET;
#endif
	setupEthernet();
}

void CommsController::update() {
#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_UDP_PROCESS;
#endif
	processUdp();

#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_USB_PROCESS;
#endif
	processUsbSerial();

#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_TX_QUEUE;
#endif
	processTxQueue();
}

bool CommsController::enqueueRx(const char* msg, const IpAddress& ip, uint16_t port) {
#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_RX_ENQUEUE;
#endif

	int next_head = (m_rxQueueHead + 1) % RX_QUEUE_SIZE;
	if (next_head == m_rxQueueTail) {
		if (m_guiDiscovered && EthernetMgr.PhyLinkActive()) {
			char errorMsg[128];
			snprintf(errorMsg, sizeof(errorMsg), "%s_ERROR: RX QUEUE OVERFLOW - COMMAND DROPPED", DEVICE_NAME_UPPER);
			m_udp.Connect(m_guiIp, m_guiPort);
			m_udp.PacketWrite(errorMsg);
			m_udp.PacketSend();
		}
		return false;
	}
	strncpy(m_rxQueue[m_rxQueueHead].buffer, msg, MAX_MESSAGE_LENGTH);
	m_rxQueue[m_rxQueueHead].buffer[MAX_MESSAGE_LENGTH - 1] = '\0';
	m_rxQueue[m_rxQueueHead].remoteIp = ip;
	m_rxQueue[m_rxQueueHead].remotePort = port;
	m_rxQueueHead = next_head;
	return true;
}

bool CommsController::dequeueRx(Message& msg) {
	if (m_rxQueueHead == m_rxQueueTail) {
		return false;
	}
	msg = m_rxQueue[m_rxQueueTail];
	m_rxQueueTail = (m_rxQueueTail + 1) % RX_QUEUE_SIZE;
	return true;
}

bool CommsController::enqueueTx(const char* msg, const IpAddress& ip, uint16_t port) {
	int next_head = (m_txQueueHead + 1) % TX_QUEUE_SIZE;
	if (next_head == m_txQueueTail) {
		if (m_guiDiscovered && EthernetMgr.PhyLinkActive()) {
			char errorMsg[128];
			snprintf(errorMsg, sizeof(errorMsg), "%s_ERROR: TX QUEUE OVERFLOW - MESSAGE DROPPED", DEVICE_NAME_UPPER);
			m_udp.Connect(m_guiIp, m_guiPort);
			m_udp.PacketWrite(errorMsg);
			m_udp.PacketSend();
		}
		return false;
	}
	strncpy(m_txQueue[m_txQueueHead].buffer, msg, MAX_MESSAGE_LENGTH);
	m_txQueue[m_txQueueHead].buffer[MAX_MESSAGE_LENGTH - 1] = '\0';
	m_txQueue[m_txQueueHead].remoteIp = ip;
	m_txQueue[m_txQueueHead].remotePort = port;
	m_txQueueHead = next_head;
	return true;
}

void CommsController::processUdp() {
	// Limit UDP packets processed per call to prevent watchdog timeout.
	const int MAX_UDP_PACKETS_PER_CALL = 1;
	int packetsProcessed = 0;
	
	while (packetsProcessed < MAX_UDP_PACKETS_PER_CALL && m_udp.PacketParse()) {
#if WATCHDOG_ENABLED
		g_watchdogBreadcrumb = WD_BREADCRUMB_UDP_PACKET_READ;
#endif
		IpAddress remoteIp = m_udp.RemoteIp();
		uint16_t remotePort = m_udp.RemotePort();
		int32_t bytesRead = m_udp.PacketRead(m_packetBuffer, MAX_PACKET_LENGTH - 1);
		if (bytesRead > 0) {
			m_packetBuffer[bytesRead] = '\0';
			if (!enqueueRx((char*)m_packetBuffer, remoteIp, remotePort)) {
				// Error handled in enqueueRx.
			}
		}
		packetsProcessed++;
	}
}

void CommsController::processUsbSerial() {
#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_USB_AVAILABLE;
#endif

	static char usbBuffer[MAX_MESSAGE_LENGTH];
	static int usbBufferIndex = 0;
	static bool usbFirstData = false;

	// Limit characters processed per call to prevent watchdog timeout.
	int charsProcessed = 0;
	const int MAX_CHARS_PER_CALL = 32;

	int available = ConnectorUsb.AvailableForRead();
	if (available > 0 && !usbFirstData) {
		g_errorLog.logf(LOG_INFO, "USB: First data seen (%d bytes)", available);
		usbFirstData = true;
	}

	// Periodic debug log if we keep getting data.
	static uint32_t lastDataLog = 0;
	if (available > 0 && (Milliseconds() - lastDataLog > 5000)) {
		g_errorLog.logf(LOG_DEBUG, "USB: %d bytes available", available);
		lastDataLog = Milliseconds();
	}

	while (ConnectorUsb.AvailableForRead() > 0 && charsProcessed < MAX_CHARS_PER_CALL) {
		char c = ConnectorUsb.CharGet();
		charsProcessed++;

		// Handle newline as message terminator.
		if (c == '\n' || c == '\r') {
			if (usbBufferIndex > 0) {
				usbBuffer[usbBufferIndex] = '\0';

				static uint32_t lastRxTime = Milliseconds();
				uint32_t now = Milliseconds();
				uint32_t timeSinceLastRx = now - lastRxTime;

				if (timeSinceLastRx > 10000) {
					g_errorLog.logf(LOG_WARNING, "USB RX after %lu ms gap: %s", timeSinceLastRx, usbBuffer);
				} else {
					g_errorLog.logf(LOG_INFO, "USB RX: %s", usbBuffer);
				}
				lastRxTime = now;

				// Mark USB host as active when we receive a command.
				notifyUsbHostActive();

				// Enqueue as if from local host (use loopback and CLIENT_PORT).
				IpAddress dummyIp(127, 0, 0, 1);
				if (!enqueueRx(usbBuffer, dummyIp, CLIENT_PORT)) {
					g_errorLog.log(LOG_ERROR, "USB RX queue overflow");
				}
				usbBufferIndex = 0;
			}
		}
		// Add character to buffer.
		else if (usbBufferIndex < MAX_MESSAGE_LENGTH - 1) {
			usbBuffer[usbBufferIndex++] = c;
		}
		// Buffer overflow protection - discard message.
		else {
			usbBufferIndex = 0;
			char errorMsg[128];
			snprintf(errorMsg, sizeof(errorMsg), "%s_ERROR: USB command too long\n", DEVICE_NAME_UPPER);
			ConnectorUsb.Send(errorMsg);
			g_errorLog.log(LOG_ERROR, "USB command too long - discarded");
		}
	}
}

void CommsController::processTxQueue() {
#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_PROCESS_TX_QUEUE;
#endif

	uint32_t now = Milliseconds();
	int usbAvail = ConnectorUsb.AvailableForWrite();

	// Heartbeat log - compact entries tracking USB and network status.
	static uint32_t lastHeartbeat = 0;
	if (now - lastHeartbeat > 30000) {
		bool networkActive = EthernetMgr.PhyLinkActive();
		uint8_t usbAvailClamped = (usbAvail > 255) ? 255 : (uint8_t)usbAvail;
		g_heartbeatLog.log(m_usbHostConnected, networkActive, usbAvailClamped);
		lastHeartbeat = now;
	}

	if (usbAvail > 5) {
		m_lastUsbHealthy = now;
		if (!m_usbHostConnected) {
#if WATCHDOG_ENABLED
			g_watchdogBreadcrumb = WD_BREADCRUMB_USB_RECONNECT;
#endif
			m_usbHostConnected = true;
			g_errorLog.logf(LOG_INFO, "USB host reconnected (buffer space: %d)", usbAvail);
			if (usbAvail > 40) {
#if WATCHDOG_ENABLED
				g_watchdogBreadcrumb = WD_BREADCRUMB_USB_SEND;
#endif
				char recoveryMsg[64];
				snprintf(recoveryMsg, sizeof(recoveryMsg), "%s_INFO: USB host reconnected\n", DEVICE_NAME_UPPER);
				ConnectorUsb.Send(recoveryMsg);
			}
#if WATCHDOG_ENABLED
			g_watchdogBreadcrumb = WD_BREADCRUMB_TX_QUEUE;
#endif
		}
	} else {
		if (m_usbHostConnected && (now - m_lastUsbHealthy) > 3000) {
			m_usbHostConnected = false;
			g_errorLog.logf(LOG_WARNING, "USB host disconnected (buffer full for 3s, space: %d)", usbAvail);
		}

		if (!m_usbHostConnected && (now - m_lastUsbHealthy) > 2000) {
			static uint32_t lastUsbResetAttempt = 0;
			if (now - lastUsbResetAttempt > 5000) {
				lastUsbResetAttempt = now;
				g_errorLog.logf(LOG_WARNING, "USB stuck for %lu ms - attempting recovery", now - m_lastUsbHealthy);

#if WATCHDOG_ENABLED
				g_watchdogBreadcrumb = WD_BREADCRUMB_USB_RECOVERY;
#endif
				uint32_t usbRecoveryStart = Milliseconds();
				ConnectorUsb.PortClose();
				while (Milliseconds() - usbRecoveryStart < 50) {
					// Spin briefly - max 50ms which is well under watchdog timeout.
				}
				ConnectorUsb.PortOpen();
#if WATCHDOG_ENABLED
				g_watchdogBreadcrumb = WD_BREADCRUMB_TX_QUEUE;
#endif
				m_lastUsbHealthy = now;
				g_errorLog.log(LOG_INFO, "USB recovery attempted - port reopened");
			}
		}
	}

	if (m_txQueueHead != m_txQueueTail) {
#if WATCHDOG_ENABLED
		g_watchdogBreadcrumb = WD_BREADCRUMB_TX_QUEUE_DEQUEUE;
#endif
		Message msg = m_txQueue[m_txQueueTail];
		m_txQueueTail = (m_txQueueTail + 1) % TX_QUEUE_SIZE;

#if WATCHDOG_ENABLED
		g_watchdogBreadcrumb = WD_BREADCRUMB_TX_QUEUE_UDP;
#endif
		IpAddress localhost(127, 0, 0, 1);
		IpAddress zeroIp(0, 0, 0, 0);
		bool isLocalhost = (msg.remoteIp == localhost);
		bool isZero = (msg.remoteIp == zeroIp);
		bool hasValidNetworkIp = !isLocalhost && !isZero;

		if (EthernetMgr.PhyLinkActive() && hasValidNetworkIp) {
			m_udp.Connect(msg.remoteIp, msg.remotePort);
			m_udp.PacketWrite(msg.buffer);
			m_udp.PacketSend();
		}

#if WATCHDOG_ENABLED
		g_watchdogBreadcrumb = WD_BREADCRUMB_TX_QUEUE_USB;
#endif
		if (m_usbHostConnected) {
			int msgLen = (int)strlen(msg.buffer);
			const int USB_STREAM_BLOCK_SIZE = 64;
			const uint32_t USB_SEND_TIMEOUT_MS = 100;
			uint32_t sendStart = Milliseconds();
			int offset = 0;

			while (offset < msgLen) {
				if (Milliseconds() - sendStart > USB_SEND_TIMEOUT_MS) break;

				int avail = ConnectorUsb.AvailableForWrite();
				if (avail <= 0) continue;

				int remaining = msgLen - offset;
				int toSend = remaining;
				if (toSend > avail) toSend = avail;
				if (toSend > USB_STREAM_BLOCK_SIZE) toSend = USB_STREAM_BLOCK_SIZE;

				char buf[USB_STREAM_BLOCK_SIZE + 1];
				memcpy(buf, msg.buffer + offset, toSend);
				buf[toSend] = '\0';
				ConnectorUsb.Send(buf);
				offset += toSend;
			}

			// Always terminate the line so partial messages don't
			// concatenate with subsequent ones.
			uint32_t nlStart = Milliseconds();
			while (ConnectorUsb.AvailableForWrite() < 1) {
				if (Milliseconds() - nlStart > 10) break;
			}
			if (ConnectorUsb.AvailableForWrite() >= 1) {
				ConnectorUsb.Send("\n");
			}
		}
	}
}

void CommsController::reportEvent(const char* statusType, const char* message) {
#if WATCHDOG_ENABLED
	uint32_t savedBreadcrumb = g_watchdogBreadcrumb;
	g_watchdogBreadcrumb = WD_BREADCRUMB_REPORT_EVENT;
#endif

	char fullMsg[MAX_MESSAGE_LENGTH];
	snprintf(fullMsg, sizeof(fullMsg), "%s%s", statusType, message);

	// Always queue messages for TX - they will be sent to both network (if GUI discovered) and USB.
	// If GUI not discovered, use dummy IP; processTxQueue will still mirror to USB.
	IpAddress targetIp = m_guiDiscovered ? m_guiIp : IpAddress(0, 0, 0, 0);
	uint16_t targetPort = m_guiDiscovered ? m_guiPort : 0;

#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_ENQUEUE_TX;
#endif
	enqueueTx(fullMsg, targetIp, targetPort);

#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = savedBreadcrumb;
#endif
}

void CommsController::notifyUsbHostActive() {
	// Called when a command is received over USB.
	// Immediately mark the host as connected and reset the health timer.
	if (!m_usbHostConnected) {
		g_errorLog.log(LOG_INFO, "USB host detected via command");

		// Clear TX queue - any messages queued while host was disconnected are stale.
		int oldQueueSize = (m_txQueueHead >= m_txQueueTail) ?
			(m_txQueueHead - m_txQueueTail) :
			(TX_QUEUE_SIZE - m_txQueueTail + m_txQueueHead);
		m_txQueueHead = 0;
		m_txQueueTail = 0;
		
		if (oldQueueSize > 0) {
			g_errorLog.logf(LOG_INFO, "Cleared %d stale TX messages", oldQueueSize);
		}

		// Clear USB input buffer to remove any stale data.
		ConnectorUsb.FlushInput();
		g_errorLog.log(LOG_DEBUG, "Flushed USB input buffer");
		
		// Queue a message to indicate USB host was detected.
		char msg[80];
		snprintf(msg, sizeof(msg), "%s_INFO: USB host detected via command", DEVICE_NAME_UPPER);
		IpAddress dummyIp(127, 0, 0, 1);
		enqueueTx(msg, dummyIp, 0);
	}
	m_usbHostConnected = true;
	m_lastUsbHealthy = Milliseconds();
}

void CommsController::setupUsbSerial(void) {
	ConnectorUsb.Mode(Connector::USB_CDC);
	ConnectorUsb.Speed(9600);
	ConnectorUsb.PortOpen();
	// USB setup is non-blocking - the connector will become available when ready.

	g_errorLog.log(LOG_INFO, "USB serial port opened");
}

void CommsController::setupEthernet() {
	EthernetMgr.Setup();

	// Start DHCP but don't hang if it fails - the system can still function via USB.
#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_DHCP;
#endif
	if (!EthernetMgr.DhcpBegin()) {
		g_errorLog.log(LOG_WARNING, "DHCP failed - network unavailable");
		return;
	}

	// Wait for link with timeout (watchdog not yet enabled at this point).
#if WATCHDOG_ENABLED
	g_watchdogBreadcrumb = WD_BREADCRUMB_SETUP_LINK_WAIT;
#endif
	uint32_t link_timeout = 2000;
	uint32_t link_start = Milliseconds();
	while (!EthernetMgr.PhyLinkActive()) {
		if (Milliseconds() - link_start > link_timeout) {
			g_errorLog.log(LOG_WARNING, "Ethernet link timeout - network unavailable");
			return;
		}
		Delay_ms(10);
	}

	m_udp.Begin(LOCAL_PORT);
	g_errorLog.logf(LOG_INFO, "Network ready on port %d", LOCAL_PORT);

	char infoMsg[128];
	snprintf(infoMsg, sizeof(infoMsg), "%s_INFO: Network ready, listening on port %d\n", DEVICE_NAME_UPPER, LOCAL_PORT);
	ConnectorUsb.Send(infoMsg);
}


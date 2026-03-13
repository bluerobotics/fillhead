/**
 * @file comms_controller.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Defines the CommsController for handling network and serial communications.
 *
 * @details This class abstracts the underlying communication hardware (Ethernet and USB Serial)
 * and provides a simple, queue-based interface for sending and receiving messages. It is
 * responsible for device discovery, packetizing data, and parsing incoming commands.
 * This ensures that the main application logic remains decoupled from the specifics
 * of the communication protocols.
 */
#pragma once

#include "ClearCore.h"
#include "EthernetUdp.h"
#include "IpAddress.h"
#include "config.h"
#include "commands.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * @struct Message
 * @brief Represents a single data packet for communication.
 * @details This structure encapsulates a message payload along with its sender's
 * network information. It is used in the transmit (TX) and receive (RX) queues.
 */
struct Message {
	char buffer[MAX_MESSAGE_LENGTH]; ///< The raw message payload as a C-style string.
	IpAddress remoteIp;              ///< The IP address of the remote host (sender or recipient).
	uint16_t remotePort;             ///< The port number of the remote host.
};

/**
 * @class CommsController
 * @brief Manages all communication tasks for the device.
 * @details This class provides a high-level interface for network and serial communications.
 * It uses separate queues for incoming (RX) and outgoing (TX) messages to ensure
 * non-blocking operation. The controller handles UDP packet processing, device discovery
 * negotiation with the GUI, and parsing of command strings into a structured format.
 */
class CommsController {
	public:
	/**
     * @brief Constructs a new CommsController object.
     * @details Initializes member variables, including the head and tail pointers
     * for the RX and TX message queues, and sets the initial discovery state.
     */
	CommsController();

    /**
     * @brief Initializes the communication hardware.
     * @details Sets up the USB serial port for debugging and initializes the Ethernet
     * controller, including DHCP negotiation and UDP listener setup. This method
     * should be called once at startup.
     */
	void setup();

    /**
     * @brief The main update loop for the communications controller.
     * @details This method should be called repeatedly in the main application loop.
     * It polls for new UDP packets and processes the outgoing message queue,
     * ensuring timely communication without blocking other system tasks.
     */
	void update();

	// Queue Management
	/**
     * @brief Enqueues a received message into the RX queue.
     * @details This function is called by the UDP processing logic when a new packet
     * is received. It copies the message and sender's info into the next available
     * slot in the RX queue.
     * @param msg The raw message string to enqueue.
     * @param ip The IP address of the sender.
     * @param port The port of the sender.
     * @return true if the message was successfully enqueued.
     * @return false if the RX queue is full, in which case the message is dropped.
     * @note If the queue is full, an error message is immediately sent back to the GUI.
     */
	bool enqueueRx(const char* msg, const IpAddress& ip, uint16_t port);

    /**
     * @brief Dequeues a message from the RX queue for processing.
     * @details The main application logic calls this function to retrieve the oldest
     * message from the RX queue.
     * @param[out] msg A reference to a `Message` struct to be populated with the
     *                 dequeued message data.
     * @return true if a message was successfully dequeued.
     * @return false if the RX queue is empty.
     */
	bool dequeueRx(Message& msg);

    /**
     * @brief Enqueues a message into the TX queue to be sent.
     * @details This function is used by the application to send messages to a remote
     * host. It copies the message and destination info into the TX queue. The
     * `update()` method will handle the actual transmission.
     * @param msg The raw message string to send.
     * @param ip The IP address of the destination.
     * @param port The port of the destination.
     * @return true if the message was successfully enqueued.
     * @return false if the TX queue is full, in which case the message is dropped.
     * @note If the queue is full, an error message is immediately sent back to the GUI.
     */
	bool enqueueTx(const char* msg, const IpAddress& ip, uint16_t port);
	
	/**
     * @brief A helper function to enqueue a formatted status or event message.
     * @details This is a convenience wrapper around `enqueueTx` that standardizes
     * the format for system events (e.g., INFO, ERROR, DONE). It sends the
     * message to the discovered GUI application.
     * @param statusType A string prefix indicating the message type (e.g., "INFO: ").
     * @param message The main content of the message.
     */
	void reportEvent(const char* statusType, const char* message);

    /**
     * @brief Parses a raw command string into a `Command` enum.
     * @details This function takes a raw string received from the network and
     * converts it into a structured `Command` enum, which can be easily used
     * in a switch-case statement by the main application logic.
     * @param msg The raw command string to parse.
     * @return The corresponding `Command` enum value, or `CMD_UNKNOWN` if the
     *         command is not recognized.
     */
	Command parseCommand(const char* msg);

	// Getters
	/**
     * @brief Checks if the GUI application has been discovered.
     * @return true if the GUI has been discovered.
     */
	bool isGuiDiscovered() const { return m_guiDiscovered; }

    /**
     * @brief Gets the IP address of the discovered GUI.
     * @return The `IpAddress` of the GUI.
     */
	IpAddress getGuiIp() const { return m_guiIp; }

    /**
     * @brief Gets the port number of the discovered GUI.
     * @return The port number of the GUI.
     */
	uint16_t getGuiPort() const { return m_guiPort; }

	// Setters
	/**
     * @brief Sets the discovery state of the GUI.
     * @param discovered The new discovery state.
     */
	void setGuiDiscovered(bool discovered) { m_guiDiscovered = discovered; }

    /**
     * @brief Sets the IP address of the GUI.
     * @param ip The new IP address.
     */
	void setGuiIp(const IpAddress& ip) { m_guiIp = ip; }

    /**
     * @brief Sets the port number of the GUI.
     * @param port The new port number.
     */
	void setGuiPort(uint16_t port) { m_guiPort = port; }
	
	/**
	 * @brief Notifies the comms controller that USB host has sent data.
	 * @details Called when a command is received over USB. This marks the
	 * USB host as connected and resets the health check timer.
	 */
	void notifyUsbHostActive();

	private:
	/**
     * @brief Processes incoming UDP packets.
     * @details Reads any available data from the UDP buffer and enqueues it
     * into the RX queue for later processing.
     */
	void processUdp();

    /**
     * @brief Processes incoming USB serial data.
     * @details Reads characters from the USB serial port, buffers them until
     * a newline is received, then enqueues the complete message into the RX queue.
     */
	void processUsbSerial();

    /**
     * @brief Configures and initializes the Ethernet hardware.
     * @details This function handles the low-level setup of the Ethernet manager,
     * including DHCP negotiation to obtain an IP address and starting the UDP listener.
     */
	void setupEthernet();

    /**
     * @brief Processes the outgoing message queue.
     * @details Dequeues one message from the TX queue (if available) and sends it
     * over UDP and/or USB to its destination(s), while honoring USB backpressure.
     */
	void processTxQueue();
	
    /**
     * @brief Configures and initializes the USB serial port.
     * @details This function sets up the USB port to act as a CDC (serial) device,
     * which is useful for debugging purposes.
     */
	void setupUsbSerial();

	EthernetUdp m_udp;          ///< The underlying UDP communication object.
	IpAddress m_guiIp;          ///< The IP address of the remote GUI application.
	uint16_t m_guiPort;         ///< The port number of the remote GUI application.
	bool m_guiDiscovered;       ///< Flag indicating if a handshake with the GUI has occurred.

	unsigned char m_packetBuffer[MAX_PACKET_LENGTH]; ///< Buffer for reading raw UDP data.
	
	// Message Queues
	Message m_rxQueue[RX_QUEUE_SIZE];   ///< The circular buffer for incoming messages.
	volatile int m_rxQueueHead;         ///< Index of the next free slot in the RX queue.
	volatile int m_rxQueueTail;         ///< Index of the next message to be read from the RX queue.

	Message m_txQueue[TX_QUEUE_SIZE];   ///< The circular buffer for outgoing messages.
	volatile int m_txQueueHead;         ///< Index of the next free slot in the TX queue.
	volatile int m_txQueueTail;         ///< Index of the next message to be sent from the TX queue.
	
	// USB host health tracking
	uint32_t m_lastUsbHealthy;
	bool m_usbHostConnected;
};

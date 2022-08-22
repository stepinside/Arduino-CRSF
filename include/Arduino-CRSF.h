#pragma once

#include <Arduino.h>

// ELRS
/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

// Basic setup
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_CONNECTION_TIMEOUT 1000

// Device address & type, The address means the destination of the data packet, so for decoder, the destination is the FC.
#define CRSF_TYPE_SETTINGS_WRITE 0x2D
#define CRSF_ADDRESS_MODULE 0xEE            // Crossfire transmitter
#define CRSF_ADDRESS_RADIO 0xEA             // Radio Transmitter
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 // Flight Controler

// Define channel input limite
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 992
#define CRSF_CHANNEL_MAX 1810

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US 1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US 6667   // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz
#define CRSF_PACKET_TIMEOUT_US 100000
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX 60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE 26
#define CRSF_FRAME_LENGTH 24 // length of type + payload + crc

class CRSF
{
private:
    HardwareSerial *m_port;

    uint8_t m_inBuffer[CRSF_PACKET_SIZE];
    uint8_t m_crsfData[CRSF_PACKET_SIZE];
    uint16_t m_channels[CRSF_MAX_CHANNEL];
    unsigned long m_lastPacketReceived;
    bool m_disconnected;

    void (*m_dataReceivedCallback)(const uint16_t channels[]);
    void (*m_disconnectedCallback)();

    void updateChannels();
    uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) const;

public:
    CRSF();
    ~CRSF();

    void readPacket();

    void begin(HardwareSerial *port, unsigned long baud = 420000);
    uint16_t getChannel(uint8_t channel) const;
    bool isConnected();


    void onDataReceived(void (*pCallback)(const uint16_t channels[]));
    void onDisconnected(void (*pCallback)());
};

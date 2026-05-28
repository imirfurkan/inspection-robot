/*
 * U2D2 Serial Link Test
 * ======================
 * Verifies the communication chain: RPi → USB → U2D2 → Dynamixel bus
 * Pings all IDs (1-10) to discover connected Dynamixel motors.
 *
 * Wiring:
 *   U2D2 USB → RPi USB port
 *   U2D2 Dynamixel port → daisy-chained motors via JST cables
 *   U2D2 Power Hub → LiPo battery (motors need external power)
 *
 * Prerequisites:
 *   # Install Dynamixel SDK
 *   cd ~
 *   git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
 *   cd DynamixelSDK/c++/build/linux_sbc
 *   make
 *   sudo make install
 *
 *   # Find U2D2 port
 *   ls /dev/ttyUSB*
 *
 * Compile:
 *   g++ -o test_u2d2_link test_u2d2_link.cpp -ldxl_x64_cpp
 *
 *   # On RPi5 (aarch64):
 *   g++ -o test_u2d2_link test_u2d2_link.cpp -I/usr/local/include/dynamixel_sdk -ldxl_sbc_cpp
 *
 * Usage:
 *   sudo ./test_u2d2_link
 *   sudo ./test_u2d2_link /dev/ttyUSB0    # specify port
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include "dynamixel_sdk.h"

// Protocol 2.0 for XH540 series
constexpr int PROTOCOL_VERSION = 2;

// Default settings
constexpr char DEFAULT_PORT[] = "/dev/ttyUSB0";
constexpr int  BAUDRATE = 57600;  // factory default for XH540

// Dynamixel XH540-W140 control table addresses
constexpr uint16_t ADDR_MODEL_NUMBER    = 0;
constexpr uint16_t ADDR_FIRMWARE_VER    = 6;
constexpr uint16_t ADDR_OPERATING_MODE  = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE   = 64;
constexpr uint16_t ADDR_PRESENT_VOLTAGE = 144;
constexpr uint16_t ADDR_PRESENT_TEMP    = 146;

// Scan range
constexpr int SCAN_ID_MIN = 1;
constexpr int SCAN_ID_MAX = 10;


int main(int argc, char* argv[])
{
    const char* port = (argc > 1) ? argv[1] : DEFAULT_PORT;

    std::cout << "=================================================\n";
    std::cout << "  U2D2 Link Test — Dynamixel Bus Scanner\n";
    std::cout << "  Port: " << port << "  Baud: " << BAUDRATE << "\n";
    std::cout << "=================================================\n\n";

    // ── Open port ──
    auto* portHandler = dynamixel::PortHandler::getPortHandler(port);
    auto* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        std::cerr << "  Failed to open port " << port << "\n";
        std::cerr << "  Check:\n";
        std::cerr << "    - U2D2 connected? (ls /dev/ttyUSB*)\n";
        std::cerr << "    - Permission? (sudo or add user to dialout group)\n";
        return 1;
    }
    std::cout << "  Port opened.\n";

    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "  Failed to set baud rate.\n";
        portHandler->closePort();
        return 1;
    }
    std::cout << "  Baud rate set to " << BAUDRATE << ".\n\n";

    // ── Scan for motors ──
    std::cout << "  Scanning IDs " << SCAN_ID_MIN << "-" << SCAN_ID_MAX << "...\n\n";

    int found = 0;

    for (int id = SCAN_ID_MIN; id <= SCAN_ID_MAX; id++) {
        uint8_t dxl_error = 0;
        int result = packetHandler->ping(portHandler, id, &dxl_error);

        if (result != COMM_SUCCESS) {
            continue;  // no response from this ID
        }

        found++;

        // Read model number
        uint16_t model = 0;
        packetHandler->read2ByteTxRx(portHandler, id, ADDR_MODEL_NUMBER, &model, &dxl_error);

        // Read firmware version
        uint8_t fw = 0;
        packetHandler->read1ByteTxRx(portHandler, id, ADDR_FIRMWARE_VER, &fw, &dxl_error);

        // Read operating mode
        uint8_t op_mode = 0;
        packetHandler->read1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, &op_mode, &dxl_error);

        // Read voltage (unit: 0.1V)
        uint16_t raw_voltage = 0;
        packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_VOLTAGE, &raw_voltage, &dxl_error);
        float voltage = raw_voltage / 10.0f;

        // Read temperature (°C)
        uint8_t temp = 0;
        packetHandler->read1ByteTxRx(portHandler, id, ADDR_PRESENT_TEMP, &temp, &dxl_error);

        // Read torque status
        uint8_t torque = 0;
        packetHandler->read1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, &torque, &dxl_error);

        // Decode operating mode
        const char* mode_str = "unknown";
        switch (op_mode) {
            case 0:  mode_str = "current control";       break;
            case 1:  mode_str = "velocity control";      break;
            case 3:  mode_str = "position control";      break;
            case 4:  mode_str = "extended position";     break;
            case 5:  mode_str = "current-based position"; break;
            case 16: mode_str = "PWM control";           break;
        }

        // Decode model
        const char* model_str = "unknown";
        if (model == 1120) model_str = "XH540-W140";
        else if (model == 1110) model_str = "XH540-W270";
        else if (model == 1060) model_str = "XH540-V150";
        else if (model == 1050) model_str = "XH540-V270";

        std::cout << "  ┌─ ID " << id << " ─────────────────────────\n";
        std::cout << "  │ Model:     " << model_str << " (" << model << ")\n";
        std::cout << "  │ Firmware:  v" << (int)fw << "\n";
        std::cout << "  │ Mode:      " << mode_str << " (" << (int)op_mode << ")\n";
        std::cout << "  │ Voltage:   " << voltage << "V\n";
        std::cout << "  │ Temp:      " << (int)temp << "°C\n";
        std::cout << "  │ Torque:    " << (torque ? "ENABLED" : "disabled") << "\n";
        std::cout << "  └────────────────────────────────\n\n";
    }

    if (found == 0) {
        std::cout << "  No motors found.\n";
        std::cout << "  Check:\n";
        std::cout << "    - Power Hub connected to battery and switched ON?\n";
        std::cout << "    - JST cables between U2D2 and motors?\n";
        std::cout << "    - Baud rate matches motor config? (default 57600)\n";
        std::cout << "    - Motor IDs in range " << SCAN_ID_MIN << "-" << SCAN_ID_MAX << "?\n";
    } else {
        std::cout << "  Found " << found << " motor(s) on the bus.\n";
    }

    portHandler->closePort();
    return 0;
}

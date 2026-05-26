/*
 * Dynamixel XH540-W140-R Motor Test
 * ===================================
 * Basic motor control test: velocity mode (forward/backward/stop)
 * and position mode (go to angle).
 *
 * This assumes motors are already discovered by test_u2d2_link.
 *
 * IMPORTANT: Ensure the robot is on a stand or wheels are off the ground
 * before running velocity tests!
 *
 * Prerequisites:
 *   Dynamixel SDK installed (see test_u2d2_link.cpp)
 *
 * Compile:
 *   g++ -o test_dynamixel test_dynamixel.cpp -ldxl_x64_cpp
 *   # On RPi5 (aarch64):
 *   g++ -o test_dynamixel test_dynamixel.cpp -I/usr/local/include/dynamixel_sdk -ldxl_sbc_cpp
 *
 * Usage:
 *   sudo ./test_dynamixel
 *   sudo ./test_dynamixel /dev/ttyUSB1    # specify port
 */

#include <iostream>
#include <vector>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include "dynamixel_sdk.h"

// ── Protocol & port ──────────────────────────────────────────────────

constexpr int PROTOCOL_VERSION = 2;
constexpr char DEFAULT_PORT[]  = "/dev/ttyUSB0";
constexpr int  BAUDRATE        = 57600;

// ── XH540-W140 control table ────────────────────────────────────────

constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_LED              = 65;
constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;
constexpr uint16_t ADDR_GOAL_POSITION    = 116;
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;
constexpr uint16_t ADDR_PRESENT_VOLTAGE  = 144;
constexpr uint16_t ADDR_PRESENT_TEMP     = 146;

// Operating modes
constexpr uint8_t MODE_VELOCITY = 1;
constexpr uint8_t MODE_POSITION = 3;

// Velocity units: 0.229 RPM per unit
// ~100 units ≈ 23 RPM (slow and safe for testing)
constexpr int32_t TEST_VELOCITY_SLOW = 50;
constexpr int32_t TEST_VELOCITY_MED  = 150;
constexpr int32_t TEST_VELOCITY_FAST = 300;

// Position: 0-4095 for 0-360°
// Center position
constexpr int32_t POS_CENTER = 2048;
constexpr int32_t POS_LEFT   = 1024;  // ~90°
constexpr int32_t POS_RIGHT  = 3072;  // ~270°


// ── Globals for signal handling ─────────────────────────────────────

dynamixel::PortHandler*   g_port   = nullptr;
dynamixel::PacketHandler* g_packet = nullptr;
std::vector<int>          g_motor_ids;


void emergencyStop()
{
    if (!g_port || !g_packet) return;

    for (int id : g_motor_ids) {
        uint8_t err = 0;
        // Set velocity to 0
        g_packet->write4ByteTxRx(g_port, id, ADDR_GOAL_VELOCITY, 0, &err);
        // Disable torque
        g_packet->write1ByteTxRx(g_port, id, ADDR_TORQUE_ENABLE, 0, &err);
        // LED off
        g_packet->write1ByteTxRx(g_port, id, ADDR_LED, 0, &err);
    }
    std::cout << "\n  EMERGENCY STOP — all motors disabled.\n";
}


void signalHandler(int signum)
{
    emergencyStop();
    if (g_port) g_port->closePort();
    exit(0);
}


// ── Helper functions ────────────────────────────────────────────────

bool setOperatingMode(int id, uint8_t mode)
{
    uint8_t err = 0;

    // Must disable torque before changing mode
    g_packet->write1ByteTxRx(g_port, id, ADDR_TORQUE_ENABLE, 0, &err);
    usleep(50000);

    int res = g_packet->write1ByteTxRx(g_port, id, ADDR_OPERATING_MODE, mode, &err);
    usleep(50000);

    if (res != COMM_SUCCESS) {
        std::cerr << "  Failed to set operating mode for ID " << id << "\n";
        return false;
    }

    // Re-enable torque
    g_packet->write1ByteTxRx(g_port, id, ADDR_TORQUE_ENABLE, 1, &err);
    return true;
}


void setAllVelocity(int32_t velocity)
{
    uint8_t err = 0;
    for (int id : g_motor_ids) {
        g_packet->write4ByteTxRx(g_port, id, ADDR_GOAL_VELOCITY,
                                 (uint32_t)velocity, &err);
    }
}


void setAllPosition(int32_t position)
{
    uint8_t err = 0;
    for (int id : g_motor_ids) {
        g_packet->write4ByteTxRx(g_port, id, ADDR_GOAL_POSITION,
                                 (uint32_t)position, &err);
    }
}


void printMotorStatus()
{
    std::cout << "\n  ── Motor Status ──\n";
    for (int id : g_motor_ids) {
        uint8_t err = 0;

        uint32_t raw_pos = 0;
        g_packet->read4ByteTxRx(g_port, id, ADDR_PRESENT_POSITION, &raw_pos, &err);

        uint32_t raw_vel = 0;
        g_packet->read4ByteTxRx(g_port, id, ADDR_PRESENT_VELOCITY, &raw_vel, &err);
        int32_t vel = (int32_t)raw_vel;

        uint16_t raw_v = 0;
        g_packet->read2ByteTxRx(g_port, id, ADDR_PRESENT_VOLTAGE, &raw_v, &err);

        uint8_t temp = 0;
        g_packet->read1ByteTxRx(g_port, id, ADDR_PRESENT_TEMP, &temp, &err);

        float deg = (int32_t)raw_pos * 360.0f / 4096.0f;
        float rpm = vel * 0.229f;

        std::cout << "  ID " << id
                  << " | Pos: " << (int32_t)raw_pos << " (" << deg << "°)"
                  << " | Vel: " << rpm << " RPM"
                  << " | " << raw_v / 10.0f << "V"
                  << " | " << (int)temp << "°C\n";
    }
    std::cout << "\n";
}


// ── Test routines ───────────────────────────────────────────────────

void testVelocity()
{
    std::cout << "\n  === Velocity Mode Test ===\n";
    std::cout << "  WARNING: Motors will spin! Ensure wheels are clear.\n\n";

    // Switch all to velocity mode
    for (int id : g_motor_ids) {
        if (!setOperatingMode(id, MODE_VELOCITY)) return;
    }

    while (true) {
        std::cout << "  1. All forward (slow)\n";
        std::cout << "  2. All forward (medium)\n";
        std::cout << "  3. All forward (fast)\n";
        std::cout << "  4. All backward (slow)\n";
        std::cout << "  5. All backward (medium)\n";
        std::cout << "  6. All backward (fast)\n";
        std::cout << "  7. Left spin (left backward, right forward)\n";
        std::cout << "  8. Right spin (left forward, right backward)\n";
        std::cout << "  9. Print status\n";
        std::cout << "  0. Stop and return to menu\n";
        std::cout << "  Select: ";

        int choice;
        std::cin >> choice;
        std::cin.ignore();

        uint8_t err = 0;

        switch (choice) {
            case 1: setAllVelocity(TEST_VELOCITY_SLOW);  std::cout << "  Forward slow\n"; break;
            case 2: setAllVelocity(TEST_VELOCITY_MED);   std::cout << "  Forward medium\n"; break;
            case 3: setAllVelocity(TEST_VELOCITY_FAST);  std::cout << "  Forward fast\n"; break;
            case 4: setAllVelocity(-TEST_VELOCITY_SLOW); std::cout << "  Backward slow\n"; break;
            case 5: setAllVelocity(-TEST_VELOCITY_MED);  std::cout << "  Backward medium\n"; break;
            case 6: setAllVelocity(-TEST_VELOCITY_FAST); std::cout << "  Backward fast\n"; break;
            case 7:
                // Assumes left motors are odd IDs, right motors are even IDs
                // Adjust based on your motor ID assignment
                for (int id : g_motor_ids) {
                    int32_t v = (id % 2 == 1) ? -TEST_VELOCITY_MED : TEST_VELOCITY_MED;
                    g_packet->write4ByteTxRx(g_port, id, ADDR_GOAL_VELOCITY, (uint32_t)v, &err);
                }
                std::cout << "  Left spin\n";
                break;
            case 8:
                for (int id : g_motor_ids) {
                    int32_t v = (id % 2 == 1) ? TEST_VELOCITY_MED : -TEST_VELOCITY_MED;
                    g_packet->write4ByteTxRx(g_port, id, ADDR_GOAL_VELOCITY, (uint32_t)v, &err);
                }
                std::cout << "  Right spin\n";
                break;
            case 9: printMotorStatus(); break;
            case 0:
                setAllVelocity(0);
                std::cout << "  Stopped.\n";
                return;
            default:
                std::cout << "  Unknown option\n";
        }
    }
}


void testPosition()
{
    std::cout << "\n  === Position Mode Test ===\n";
    std::cout << "  Motors will move to specific angles.\n\n";

    // Switch all to position mode
    for (int id : g_motor_ids) {
        if (!setOperatingMode(id, MODE_POSITION)) return;
    }

    while (true) {
        std::cout << "  1. Center (180°)\n";
        std::cout << "  2. Left (90°)\n";
        std::cout << "  3. Right (270°)\n";
        std::cout << "  4. Custom angle\n";
        std::cout << "  5. Print status\n";
        std::cout << "  0. Return to menu\n";
        std::cout << "  Select: ";

        int choice;
        std::cin >> choice;
        std::cin.ignore();

        switch (choice) {
            case 1:
                setAllPosition(POS_CENTER);
                std::cout << "  Moving to center (180°)\n";
                break;
            case 2:
                setAllPosition(POS_LEFT);
                std::cout << "  Moving to left (90°)\n";
                break;
            case 3:
                setAllPosition(POS_RIGHT);
                std::cout << "  Moving to right (270°)\n";
                break;
            case 4: {
                std::cout << "  Enter angle (0-360): ";
                float angle;
                std::cin >> angle;
                std::cin.ignore();
                if (angle < 0 || angle > 360) {
                    std::cout << "  Invalid angle.\n";
                    break;
                }
                int32_t pos = (int32_t)(angle / 360.0f * 4096.0f);
                setAllPosition(pos);
                std::cout << "  Moving to " << angle << "° (raw: " << pos << ")\n";
                break;
            }
            case 5: printMotorStatus(); break;
            case 0: return;
            default: std::cout << "  Unknown option\n";
        }

        // Wait a moment for the motors to move
        usleep(500000);
    }
}


void testLED()
{
    std::cout << "\n  === LED Test ===\n";
    std::cout << "  Toggling motor LEDs...\n";

    uint8_t err = 0;
    for (int id : g_motor_ids) {
        g_packet->write1ByteTxRx(g_port, id, ADDR_LED, 1, &err);
        std::cout << "  ID " << id << " LED ON\n";
        usleep(500000);
    }

    sleep(2);

    for (int id : g_motor_ids) {
        g_packet->write1ByteTxRx(g_port, id, ADDR_LED, 0, &err);
    }
    std::cout << "  All LEDs OFF\n";
}


// ── Main ────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    const char* port = (argc > 1) ? argv[1] : DEFAULT_PORT;

    // Register signal handlers for safe shutdown
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=================================================\n";
    std::cout << "  Dynamixel XH540-W140 Motor Test\n";
    std::cout << "  Port: " << port << "  Baud: " << BAUDRATE << "\n";
    std::cout << "=================================================\n\n";

    g_port   = dynamixel::PortHandler::getPortHandler(port);
    g_packet = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!g_port->openPort()) {
        std::cerr << "  Failed to open port.\n";
        return 1;
    }
    g_port->setBaudRate(BAUDRATE);

    // ── Discover motors ──
    std::cout << "  Scanning for motors...\n";
    for (int id = 1; id <= 10; id++) {
        uint8_t err = 0;
        if (g_packet->ping(g_port, id, &err) == COMM_SUCCESS) {
            g_motor_ids.push_back(id);
            std::cout << "  Found motor ID " << id << "\n";
        }
    }

    if (g_motor_ids.empty()) {
        std::cerr << "  No motors found. Run test_u2d2_link first.\n";
        g_port->closePort();
        return 1;
    }

    std::cout << "\n  " << g_motor_ids.size() << " motor(s) ready.\n\n";

    // ── Main menu ──
    while (true) {
        std::cout << "  === Main Menu ===\n";
        std::cout << "  1. Velocity test (spinning wheels)\n";
        std::cout << "  2. Position test (go to angle)\n";
        std::cout << "  3. Motor LED test\n";
        std::cout << "  4. Print motor status\n";
        std::cout << "  0. Emergency stop and exit\n";
        std::cout << "  Select: ";

        int choice;
        std::cin >> choice;
        std::cin.ignore();

        switch (choice) {
            case 1: testVelocity(); break;
            case 2: testPosition(); break;
            case 3: testLED();      break;
            case 4: printMotorStatus(); break;
            case 0:
                emergencyStop();
                g_port->closePort();
                std::cout << "  Port closed. Bye.\n";
                return 0;
            default:
                std::cout << "  Unknown option\n";
        }
        std::cout << "\n";
    }
}

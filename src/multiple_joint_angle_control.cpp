#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#define COMMAND_TOPIC "rt/arm_Command"
#define FEEDBACK_TOPIC "current_servo_angle"

using namespace unitree::robot;

using Pose = std::array<double, 7>;

std::mutex feedback_mutex;
Pose current_pose{};
bool feedback_received = false;

int sequence_number = 1;

// ---------------------------------------------------------
// Verified poses
// ---------------------------------------------------------

const Pose HOME = {
    -0.6,
    -91.0,
    88.5,
    -90.5,
    -2.0,
    89.6,
    -19.0};

const Pose LOW_RIGHT = {
    1.3,
    -91.0,
    88.4,
    -89.5,
    96.0,
    95.6,
    -19.0};

const Pose MID_RIGHT = {
    -0.7,
    -59.6,
    35.5,
    -87.4,
    93.6,
    64.9,
    -19.0};

const Pose HIGH_RIGHT = {
    -0.7,
    -5.3,
    -88.5,
    -90.8,
    90.4,
    -3.2,
    -19.0};

// Swivel wrist values (J4 and J5)
const double LEFT_J4 = -87.1;
const double LEFT_J5 = 85.3;

const double MIDDLE_J4 = -2.0;
const double MIDDLE_J5 = 89.6;

const double RIGHT_J4 = 96.0;
const double RIGHT_J5 = 95.6;

// Gripper values (J6)
const double GRIP_OPEN = 49.7;    // max open with disabled torque is 70.9, but servos cant go past 49.7
const double GRIP_CLOSED = -19.0; // gripper fully closed
// const double GRIP_HOLD = ;        // gripper closed just enough to hold target

// ---------------------------------------------------------
// Feedback callback
// ---------------------------------------------------------

void FeedbackHandler(const void *msg)
{
    const auto *servo =
        static_cast<const unitree_arm::msg::dds_::PubServoInfo_ *>(msg);

    std::lock_guard<std::mutex> lock(feedback_mutex);

    current_pose = {
        servo->servo0_data_(),
        servo->servo1_data_(),
        servo->servo2_data_(),
        servo->servo3_data_(),
        servo->servo4_data_(),
        servo->servo5_data_(),
        servo->servo6_data_()};

    feedback_received = true;
}

// ---------------------------------------------------------
// Send pose
// ---------------------------------------------------------

void SendPose(
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> &publisher,
    const Pose &pose)
{
    std::ostringstream json;

    json << "{"
         << "\"seq\":" << sequence_number++ << ","
         << "\"address\":1,"
         << "\"funcode\":2,"
         << "\"data\":{"
         << "\"mode\":1,"
         << "\"angle0\":" << pose[0] << ","
         << "\"angle1\":" << pose[1] << ","
         << "\"angle2\":" << pose[2] << ","
         << "\"angle3\":" << pose[3] << ","
         << "\"angle4\":" << pose[4] << ","
         << "\"angle5\":" << pose[5] << ","
         << "\"angle6\":" << pose[6]
         << "}}";

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = json.str();

    publisher.Write(msg);

    std::cout << "Command sent: " << json.str() << std::endl;
}

// ---------------------------------------------------------
// Enable / disable
// ---------------------------------------------------------

void SetEnable(
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> &publisher,
    bool enable)
{
    unitree_arm::msg::dds_::ArmString_ msg{};

    std::ostringstream json;

    json << "{"
         << "\"seq\":" << sequence_number++ << ","
         << "\"address\":1,"
         << "\"funcode\":5,"
         << "\"data\":{\"mode\":" << (enable ? 1 : 0) << "}"
         << "}";

    msg.data_() = json.str();

    publisher.Write(msg);

    std::cout
        << (enable ? "Arm enabled." : "ARM TORQUE DISABLED.")
        << std::endl;
}

// ---------------------------------------------------------
// Check whether pose reached
// ---------------------------------------------------------

double MaxError(const Pose &target)
{
    std::lock_guard<std::mutex> lock(feedback_mutex);

    double max_error = 0.0;

    for (int i = 0; i < 7; ++i)
    {
        double error = std::abs(current_pose[i] - target[i]);

        if (error > max_error)
            max_error = error;
    }

    return max_error;
}

bool WaitForPose(
    const Pose &target,
    double tolerance = 2.0,
    double timeout_seconds = 10.0)
{
    const auto start = std::chrono::steady_clock::now();

    while (true)
    {
        if (feedback_received)
        {
            double error = MaxError(target);

            std::cout
                << "\rMax joint error: "
                << error
                << " deg    "
                << std::flush;

            if (error <= tolerance)
            {
                std::cout << "\nPose reached." << std::endl;
                return true;
            }
        }

        auto now = std::chrono::steady_clock::now();

        double elapsed =
            std::chrono::duration<double>(now - start).count();

        if (elapsed >= timeout_seconds)
        {
            std::cout
                << "\nTimeout waiting for pose."
                << std::endl;

            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }
}

// ---------------------------------------------------------
// Gripper open and closed while keeping the current pose
// ---------------------------------------------------------
void ToggleGripper(
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> &publisher)
{
    Pose target;

    {
        std::lock_guard<std::mutex> lock(feedback_mutex);

        if (!feedback_received)
        {
            std::cout << "Cannot control gripper: no joint feedback."
                      << std::endl;
            return;
        }

        // Keep the CURRENT pose exactly as measured
        target = current_pose;
    }

    double current_gripper = target[6];

    // Determine whether we're currently closer to OPEN or CLOSED
    double distance_to_open =
        std::abs(current_gripper - GRIP_OPEN);

    double distance_to_closed =
        std::abs(current_gripper - GRIP_CLOSED);

    if (distance_to_closed < distance_to_open)
    {
        // Currently closed -> open
        target[6] = GRIP_OPEN;

        std::cout << "Opening gripper..." << std::endl;
    }
    else
    {
        // Currently open -> close
        target[6] = GRIP_CLOSED;

        std::cout << "Closing gripper..." << std::endl;
    }

    SendPose(publisher, target);

    WaitForPose(target, 2.0, 5.0);
}

// ---------------------------------------------------------
// Move to named pose
// ---------------------------------------------------------

void MoveTo(
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> &publisher,
    const std::string &name,
    const Pose &pose)
{
    Pose target = pose;

    {
        std::lock_guard<std::mutex> lock(feedback_mutex);

        if (!feedback_received)
        {
            std::cout
                << "Cannot move: no joint feedback received."
                << std::endl;
            return;
        }

        // Preserve current gripper position
        target[6] = current_pose[6];
    }

    std::cout
        << "\nMoving to "
        << name
        << " while preserving gripper position..."
        << std::endl;

    SendPose(publisher, target);

    WaitForPose(target);
}

// ---------------------------------------------------------
// Print current pose
// ---------------------------------------------------------

void PrintCurrentPose()
{
    std::lock_guard<std::mutex> lock(feedback_mutex);

    if (!feedback_received)
    {
        std::cout << "No joint feedback received yet." << std::endl;
        return;
    }

    std::cout << "\nCurrent pose:\n";

    for (int i = 0; i < 7; ++i)
    {
        std::cout
            << "J" << i
            << " = "
            << current_pose[i]
            << "\n";
    }
}

void SwivelTo(
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> &publisher,
    const std::string &name,
    double j4,
    double j5)
{
    Pose target;

    {
        std::lock_guard<std::mutex> lock(feedback_mutex);

        if (!feedback_received)
        {
            std::cout << "Cannot swivel: no joint feedback."
                      << std::endl;
            return;
        }

        // Start from ACTUAL current arm pose.
        // This preserves:
        // J0-J3 = arm height/position
        // J6    = gripper position
        target = current_pose;
    }

    // Change only swivel/orientation joints
    target[4] = j4;
    target[5] = j5;

    std::cout << "\nSwivelling " << name << "..."
              << std::endl;

    SendPose(publisher, target);

    WaitForPose(target, 2.0, 10.0);
}

// ---------------------------------------------------------
// Main
// ---------------------------------------------------------

int main()
{
    ChannelFactory::Instance()->Init(0);

    ChannelPublisher<unitree_arm::msg::dds_::ArmString_>
        publisher(COMMAND_TOPIC);

    publisher.InitChannel();

    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>
        subscriber(FEEDBACK_TOPIC);

    subscriber.InitChannel(FeedbackHandler);

    std::cout << "\nD1 POSE CONTROLLER\n";
    std::cout << "==================\n";
    std::cout << "h : HOME\n";
    std::cout << "l : LOW_RIGHT\n";
    std::cout << "m : MID_RIGHT\n";
    std::cout << "u : HIGH_RIGHT\n";
    std::cout << "1 : swivel LEFT\n";
    std::cout << "2 : swivel MIDDLE\n";
    std::cout << "3 : swivel RIGHT\n";
    std::cout << "g : toggle gripper\n";
    std::cout << "p : print current joints\n";
    std::cout << "s : hold current position\n";
    std::cout << "e : enable arm\n";
    std::cout << "x : DISABLE TORQUE\n";
    std::cout << "q : quit\n\n";

    char command;

    while (true)
    {
        std::cout << "> ";
        std::cin >> command;

        switch (command)
        {
        case 'h':
            MoveTo(publisher, "HOME", HOME);
            break;

        case 'l':
            MoveTo(publisher, "LOW_RIGHT", LOW_RIGHT);
            break;

        case 'm':
            MoveTo(publisher, "MID_RIGHT", MID_RIGHT);
            break;

        case 'u':
            MoveTo(publisher, "HIGH_RIGHT", HIGH_RIGHT);
            break;

        case '1':
            SwivelTo(
                publisher,
                "LEFT",
                LEFT_J4,
                LEFT_J5);
            break;

        case '2':
            SwivelTo(
                publisher,
                "MIDDLE",
                MIDDLE_J4,
                MIDDLE_J5);
            break;

        case '3':
            SwivelTo(
                publisher,
                "RIGHT",
                RIGHT_J4,
                RIGHT_J5);
            break;

        case 'g':
            ToggleGripper(publisher);
            break;

        case 'p':
            PrintCurrentPose();
            break;

        case 's':
        {
            Pose hold_pose;

            {
                std::lock_guard<std::mutex> lock(feedback_mutex);

                if (!feedback_received)
                {
                    std::cout
                        << "Cannot hold: no feedback."
                        << std::endl;
                    break;
                }

                hold_pose = current_pose;
            }

            std::cout << "Holding current position." << std::endl;

            SendPose(publisher, hold_pose);

            break;
        }

        case 'e':
            SetEnable(publisher, true);
            break;

        case 'x':
            std::cout
                << "WARNING: disabling torque; arm may drop."
                << std::endl;

            SetEnable(publisher, false);
            break;

        case 'q':
            std::cout << "Exiting controller." << std::endl;
            return 0;

        default:
            std::cout << "Unknown command." << std::endl;
            break;
        }
    }
}
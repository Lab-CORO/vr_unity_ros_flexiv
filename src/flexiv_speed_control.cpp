/**
 * Connect the flexiv rizon4s' RDK to ROS.
 * Control the flexiv using ROS' twists messages.
 * Based on the cartesian_impedance_control.cpp from the flexiv RDK
 * @author Jonathan Boutin
*/

// Flexiv include
#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp> // Might not need it
#include <flexiv/Utility.hpp>

#include <iostream>
#include <thread>

// ROS include
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

/// @brief
/// @param msg
/// @param i
/// @param j
//void callback(const geometry_msgs::Twist::ConstPtr& msg, int i, int j)
void callback(const geometry_msgs::Twist::ConstPtr& msg, flexiv::Robot* robot, flexiv::RobotStates& robotStates)
{
    static std::vector<double> initTcpPose;
    static std::vector<double> targetTcpPose;
    // Flag whether initial Cartesian position is set
    static bool isInitPoseSet = false;

    // Time variable
    double deltaTime = 0.01;


    // Set initial pose
    if (!isInitPoseSet)
    {
        robot->getRobotStates(robotStates);
        initTcpPose = robotStates.tcpPose;
        targetTcpPose = initTcpPose;

        // Set new linear value
        targetTcpPose[0] = initTcpPose[0] + msg->linear.z * deltaTime;
        targetTcpPose[1] = initTcpPose[1] + -msg->linear.x * deltaTime;
        targetTcpPose[2] = initTcpPose[2] + msg->linear.y * deltaTime;

        isInitPoseSet = true;
    }
    else
    {
        targetTcpPose[0] = targetTcpPose[0] + msg->linear.z * deltaTime;
        targetTcpPose[1] = targetTcpPose[1] + -msg->linear.x * deltaTime;
        targetTcpPose[2] = targetTcpPose[2] + msg->linear.y * deltaTime;
    }

    // Move the robot
    robot->sendTcpPose(targetTcpPose);

    if(robot->isFault())
    {
        ros::shutdown();
    }
}

void callback_pose(const geometry_msgs::Pose::ConstPtr& msg, flexiv::Robot* robot, flexiv::RobotStates& robotStates)
{
    static std::vector<double> initTcpPose;
    static std::vector<double> targetTcpPose;

    // Flag whether initial Cartesian position is set
    static bool isInitPoseSet = false;


        // Set initial pose
    if (!isInitPoseSet)
    {
        robot->getRobotStates(robotStates);
        initTcpPose = robotStates.tcpPose;
        targetTcpPose = initTcpPose;
        isInitPoseSet = true;
    }

    // Set the target position
    targetTcpPose[0] = msg->position.x;
    targetTcpPose[1] = msg->position.y;
    targetTcpPose[2] = msg->position.z;

    targetTcpPose[3] = msg->orientation.w;
    targetTcpPose[4] = msg->orientation.x;
    targetTcpPose[5] = msg->orientation.y;
    targetTcpPose[6] = msg->orientation.z;

    // Move the robot robot
    robot->sendTcpPose(targetTcpPose);

    if (robot->isFault())
    {
        ros::shutdown();
    }
}

/// @brief Function to display help information for the parameters of the function
void printHelp()
{
    std::cout << "Required arguments: [robot ip] [local ip]" << std::endl;
    std::cout << "  robot IP: address of the robot server" << std::endl;
    std::cout << "  local IP: address of this PC" << std::endl;
    std::cout << std::endl;
}

/**
 * @brief
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    if (argc < 3
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"}))
        {
            printHelp();
            return 1;
        }

    // // Set IP address of the robot and the PC
    std::string robotIP = argv[1];
    std::string localIP = argv[2];

    // ROS parameter Initialization
    ros::init(argc, argv, "flexiv_twist_listener");
    ros::NodeHandle n;

    // // Instantiate robot interface
    flexiv::Robot robot(robotIP, localIP);
    // // Create data struct for storing the robot states
    flexiv::RobotStates robotStates;
    // Current joint states
    sensor_msgs::JointState jointStates;
    int nbrJoints = 0;

    // ROS publish frequency
    ros::Rate loop_rate = 100;


    // Clear fault on robot if any
    if (robot.isFault())
    {
        log.warn("Fault occurred on server, trying to clear ...");
        robot.clearFault();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Check if fault again
        if (robot.isFault())
        {
            log.error("Fault cannot be cleared, exiting ...");
            return 1;
        }
        log.info("Fault on robot servier is cleared.");
    }

    // Enable the robot. Make sure the E-stop is release
    log.info("Enabling robot ...");
    robot.enable();

    // Wait for the robot to become operationnal
    int secondWaited = 0;
    while (!robot.isOperational())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if(++secondWaited == 10)
        {
            log.warn(
                "Still waiting for robot to become operationnal, please "
                "check taht the robot 1) has no fault, 2) is booted "
                "into Auto mode."
            );
        }
    }
    log.info("Robot is now operationnal.");

    // Set mode
    robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE_NRT);
    // Wait for mode to be switch
    while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE_NRT)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    // Set subscriber and publisher
        // Twist sub
    // ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("unity/flexiv/twist", 1, boost::bind(callback, _1, &robot, robotStates));
        //Pose Sub
    ros::Subscriber sub = n.subscribe<geometry_msgs::Pose>("pose",1,boost::bind(callback_pose, _1, &robot, robotStates));
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("flexiv/unity/joint_states", 10);

    while (ros::ok)
    {
        // Get the robot states
        robot.getRobotStates(robotStates);

        // Resize if needed
        nbrJoints = (int)robotStates.q.size();
        if ((int)jointStates.position.size() != nbrJoints)
        {
            jointStates.position.resize(nbrJoints);
        }

        // Set the JointStates position
        for (int i = 0; i < nbrJoints; ++i)
        {
            jointStates.position[i]  = robotStates.q[i];
        }

        pub.publish(jointStates);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // printf("test_3");

    ros::spin();
}



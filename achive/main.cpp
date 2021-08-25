#include "flexiv_include/FvrRobotClient.hpp"
#include "flexiv_include/ConnectionManager.hpp"

#include <sched.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <map>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <functional>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define PI 3.14159265
#define DegreeToRadian(x) x * PI / 180.0
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_SEC (1000000000)

constexpr double k_controlLoopInterval = 0.001;
constexpr double k_secondToMillisecond = 1000.0;
unsigned int robotDof = 7;

using namespace fvr;

class Listener
{
public:
    double arraypos[7];
    // double arrayvel[7];
public:
    void msgCallback(const sensor_msgs::JointState::ConstPtr& msg_cmd);
    double posDataTransfer(unsigned int index);
    // double velDataTransfer(unsigned int index);
};

void Listener::msgCallback(const sensor_msgs::JointState::ConstPtr& msg_cmd)
{
    // _FVR_INFO("Receive targetJntPos:");
	for (unsigned int i = 0; i < 7; i++) {
		arraypos[i] = msg_cmd->position[i];
        // arrayvel[i] = msg_cmd->velocity[i];
        // _FVR_INFO("J%d :",i);
		// _FVR_INFO("%f",arraypos[i]);
	}
}

double Listener::posDataTransfer(unsigned int index)
{
    return(arraypos[index]);
}

// double Listener::velDataTransfer(unsigned int index)
// {
//     return(arrayvel[index]);
// }

void Clock_NextInterval_ms(struct timespec& ts, uint32_t ms)
{
    ts.tv_nsec += ms * NSEC_PER_MSEC;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
}

int main(int argc, char* argv[])
{
    // create a robot client
    std::shared_ptr<FvrRobotClient> robot = std::make_shared<FvrRobotClient>();

    // create connection manager
    // ConnectionManager robotConnection(robot, argv[1], argv[2]);
    const std::string& serverAddr = "192.168.2.100";
    const std::string& clientAddr = "192.168.2.110";
    ConnectionManager robotConnection(robot, serverAddr, clientAddr);

    // create thread for running connection
    std::thread connection([&]() {
        while (true) {

            if (robotConnection.run() != true) {
                return;
            }
            if (robotConnection.robotConnected() != true) {
                _FVR_INFO("Lost connection to robot server!");
                return;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // // connect to robot server
    while (true) {
        if (robotConnection.robotConnected() == true) {
            _FVR_INFO("Connect to robot server!");
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // set mode to CTRL_MODE_JNTPOS_STREAMING
    robot->setControlMode(CTRL_MODE_JOINT_PVAT_STREAMING);

    // wait for robot to enter joint joint pos streaming mode
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getCtrlMode() != CTRL_MODE_JOINT_PVAT_STREAMING);

    // read current joint pos
    RobotStatusData robotStatus;
    robot->readRobotStatus(&robotStatus);
    Eigen::VectorXd startJntPos = Eigen::VectorXd::Zero(robotStatus.m_jntPos.size());
    for (unsigned int i = 0; i < startJntPos.size(); i++) {
        startJntPos(i) = robotStatus.m_jntPos[i];
    }

    Eigen::VectorXd targetJntPos = startJntPos;
    Eigen::VectorXd targetJntVel = Eigen::VectorXd::Zero(robotDof);

    Eigen::VectorXd curJntPos = startJntPos;
    Eigen::VectorXd curJntVel = Eigen::VectorXd::Zero(robotDof);

    Eigen::VectorXd deltaJntPos = Eigen::VectorXd::Zero(robotDof);
    Eigen::VectorXd deltaJntVel = Eigen::VectorXd::Zero(robotDof);

    Eigen::VectorXd cmdJntPos = startJntPos;
    Eigen::VectorXd cmdJntVel = Eigen::VectorXd::Zero(robotDof);
    Eigen::VectorXd cmdJntAcc = Eigen::VectorXd::Zero(robotDof);

    // trajectory parameters definition
    double trajectoryStepSize = 0.002;
    double velGain = 15;
    double maxVel = DegreeToRadian(velGain);
    double minJntPosErr = DegreeToRadian(0.1);
    double maxJntPosErr = DegreeToRadian(1);
    double cmdNum;
    int cmdJnt;

    Eigen::VectorXd timeCounts = Eigen::VectorXd::Zero(robotDof);
    Eigen::VectorXd trajDuration = Eigen::VectorXd::Zero(robotDof);

    // // create thread for receiving manual input data 
    // std::thread manualInput([&]() {
    //     while (std::cin >> cmdJnt >> cmdNum) {
    //         // get current joint pos
    //         robot->readRobotStatus(&robotStatus);
    //         for (unsigned int i = 0; i < robotDof; i++) {
    //             curJntPos(i) = robotStatus.m_jntPos[i];
    //             curJntVel(i) = robotStatus.m_jntVel[i];
    //             // Set joint position command to target joint
    //             if (i == cmdJnt)
    //             {
    //                 _FVR_INFO("Jnt:%d", cmdJnt);
    //                 _FVR_INFO("CMD:%5.2f", cmdNum);
    //                 targetJntPos(i) = DegreeToRadian(cmdNum);
    //             }
    //             timeCounts(i) = 0.0;
    //             // Estimate the trajactary duration time with maximum joint velocity
    //             trajDuration(i) = abs(targetJntPos(i)-curJntPos(i))/maxVel;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     }
    // });

    // ROS publisher to send out joint position and velocity
    ros::init(argc, argv,"flexiv_arm");
	ros::NodeHandle nodeHandler;
    ros::Publisher msg_pub = nodeHandler.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Rate loop_rate(100);
    
    // create thread for publish data to ros 
    std::thread rosDataPub([&]() {
        // _FVR_INFO("first line");
        while (ros::ok())
        {
            // _FVR_INFO("second line");
            sensor_msgs::JointState msg;

            // get current joint pos
            robot->readRobotStatus(&robotStatus);
            double array_pos[7];
            double array_vel[7];
            for (unsigned int i = 0; i < robotDof; i++) {
                array_pos[i] = robotStatus.m_jntPos[i];
                array_vel[i] = robotStatus.m_jntVel[i];
            }

            std::vector<double> array1(array_pos,array_pos+7);
            std::vector<double> array2(array_vel,array_vel+7);
            msg.position = array1;
            msg.velocity = array2;

            // _FVR_INFO("Send curJntPos:");
            // for (unsigned int i = 0; i < 7; i++) {
            //     _FVR_INFO("J%d :",i);
            //     _FVR_INFO("%f",msg.curjntpos[i]);
            // }

            msg_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    });

    // create thread for subscribe data from ros 
    std::thread rosDataSub([&]() {
        Listener listener;
        ros::Subscriber msg_sub = nodeHandler.subscribe("joint_cmds", 1000, &Listener::msgCallback, &listener);
        while (ros::ok())
        {
            // get current joint pos
            robot->readRobotStatus(&robotStatus);
            for (unsigned int i = 0; i < robotDof; i++) {
                curJntPos(i) = robotStatus.m_jntPos[i];
                curJntVel(i) = robotStatus.m_jntVel[i];

                targetJntPos(i) = listener.posDataTransfer(i);
                // _FVR_INFO("J%d :",i);
                // _FVR_INFO("%f",targetJntPos(i));
                timeCounts(i) = 0.0;
                // Estimate the trajactary duration time with maximum joint velocity
                trajDuration(i) = abs(targetJntPos(i)-curJntPos(i))/maxVel;
            }
		    ros::spinOnce();
            loop_rate.sleep();
        }
    });

    // Robot controller
    struct timespec ts2;
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    while (true) {
        // get current joint pos
        robot->readRobotStatus(&robotStatus);
        for (unsigned int i = 0; i < robotDof; i++) {
            curJntPos(i) = robotStatus.m_jntPos[i];
            curJntVel(i) = robotStatus.m_jntVel[i];

            // Calculate delta of target position with current position
            deltaJntPos(i) = targetJntPos(i)-curJntPos(i);

            // Check if exceeds the minimum joint position error
            if (abs(deltaJntPos(i)) > minJntPosErr)
            {
                timeCounts(i) += trajectoryStepSize;
                // Check if exceeds trajactary duration time
                if (timeCounts(i) >= trajDuration(i))
                {
                    // If error exceeds the maximum error, trajactary duration time will be extended
                    if (abs(deltaJntPos(i)) > maxJntPosErr)
                    {
                        trajDuration(i) = abs(deltaJntPos(i))/maxVel+trajDuration(i);
                    } else {
                        timeCounts = trajDuration;
                        cmdJntVel(i) = 0;
                    }                    
                } else {
                    // Set steady velocity to the joint
                    if (deltaJntPos(i) > 0) {
                        cmdJntVel(i) = deltaJntPos(i)*velGain; 
                        if (cmdJntVel(i) > maxVel)
                        {
                            cmdJntVel(i) = maxVel;
                        }   
                    } else {
                        cmdJntVel(i) = deltaJntPos(i)*velGain; 
                        if (cmdJntVel(i) < -maxVel)
                        {
                            cmdJntVel(i) = -maxVel;
                        }  
                    } 
                }
            } else {
                cmdJntVel(i) = 0;
            }
            cmdJntPos(i) = cmdJntPos(i) + trajectoryStepSize*cmdJntVel(i);
        }

        std::vector<double> runJntPos;
        std::vector<double> runJntVel;
        std::vector<double> runJntAcc;
        for (unsigned int i = 0; i < robotDof; i++) {
            runJntPos.push_back(cmdJntPos(i));
            runJntVel.push_back(cmdJntVel(i));
            runJntAcc.push_back(cmdJntAcc(i));
        }

        // send target joint position ,velocity and acc;
        robot->streamJointPVAT(runJntPos, runJntVel, runJntAcc, 1);

        Clock_NextInterval_ms(ts2,
            (uint32_t)(k_controlLoopInterval * k_secondToMillisecond*10));
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts2, NULL);
    }
    return 0;
}


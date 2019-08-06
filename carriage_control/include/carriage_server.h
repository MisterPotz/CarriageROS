#ifndef CARRIAGE_SERVER_H_
#define CARRIAGE_SERVER_H_

//Including some useful stuff
#include <stdlib.h>
//Custom predefined action messages
#include <carriage_control/carriageAction.h>

//включаем ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <math.h>
namespace carriage_control{

struct WheelSet{
    std::string cmd_vel;
    std::string drive_joint_command[4];
};

class Carriage_Server{
    private:
        struct Cell{
            int32_t x;
            int32_t y;
        } cell;
        struct Position{
            double x;
            double y;
        } position;
        struct WheelSets{
            WheelSet along_y;
            WheelSet along_x;
        } wheel_sets;
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<carriage_control::carriageAction> ac;
        const std::string action_name;
        const std::string robot_name;
        // create messages that are used to published feedback/result
        carriage_control::carriageFeedback feedback_;
        carriage_control::carriageResult result_;
        //client to obtain position of robot from gazebo topics
        ros::ServiceClient model_states_client;
        gazebo_msgs::GetModelState gazebo_srv;
        void getModelPosition();
        void getCell();
        void initializeVars();
        void registerCallbacks();
        void goalCB(const carriage_control::carriageGoalConstPtr &goal);
        const Cell buildTrajectoryToGoal(const Cell& goal);
        void applyTrajectory(const Cell trajectory);
        void setWheelsUp(WheelSet& wheel_set);
        void setWheelsDown(WheelSet& wheel_set);
        //function that performs delivery of robot to the next edge cell
        void spinWheels(const WheelSet& wheel_set, int cells);

    public:
        Carriage_Server(const std::string name, const std::string robot_name);
        ~Carriage_Server();
        void showCell();
        void registerWheelSets(const WheelSet& x, const WheelSet& y);
};
};

#endif
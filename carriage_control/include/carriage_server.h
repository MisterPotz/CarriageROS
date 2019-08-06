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
    public:
        Carriage_Server(std::string name, std::string robot_name);
        ~Carriage_Server();
        void showCell();
};
#endif
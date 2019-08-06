#ifndef CARRIAGE_SERVER_H_
#define CARRIAGE_SERVER_H_

//Including some useful stuff
#include <stdlib.h>
//Custom predefined action messages
#include <carriageAction.h>

//включаем ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <math.h>

class Carriage_Server{
    private:
        ros::NodeHandle n;
        actionlib::SimpleActionServer<carriage_control::carriageAction> ac;
        std::string action_name_;
        // create messages that are used to published feedback/result
        carriage_control::carriageFeedback feedback_;
        carriage_control::carriageResult result_;

    public:
        Carriage_Server();

};
#endif
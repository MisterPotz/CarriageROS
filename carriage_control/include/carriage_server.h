#ifndef CARRIAGE_SERVER_H_
#define CARRIAGE_SERVER_H_

//Including some useful stuff
#include <stdlib.h>
//Custom predefined action messages
#include <carriage_control/carriageAction.h>

//builder end executioner of straight-line trajectories
//включаем ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <StraightNavigator.h>
#include <stack>



namespace carriage_control{

class TimeControl{
    private:
        ros::Rate sleep_rate_;
    public:
        TimeControl(double rate);
        ~TimeControl();
        void wait();
};

struct WheelSet{
    ros::Publisher twist_command_pub;
    ros::Publisher drive_joint_command_pub[4];
    char axis;
};
//inhereting interface so navigation class can get current position
class Carriage_Server : public PoseGetter{
    public:
        virtual geometry_msgs::Pose getCurrentPose();
        struct Cell{
            int32_t x;
            int32_t y;
        } cell; //current cell
        Carriage_Server(const std::string name, const std::string robot_name, const std::string base_name, float _cell_size);
        ~Carriage_Server();
        void showCell();
        
        void registerWheelSets(const WheelSet& x, const WheelSet& y);
        ros::NodeHandle& getNodeHandle();
        double getCellCize();
        void setTimeControl(TimeControl* time_control);
private:
        //executes the cell_stack order
        bool moveRobot2Cell();
        //wheel_parameters
        const float upper_position = 0.0f;
        const float lower_position = -0.4f;
        const float cell_size; //total size of one cell
        TimeControl* time_control_;
        std::stack<Cell> cell_order_; //navigation order of cells to reach goal
        struct Position{
            double x;
            double y;
        } position;
        struct WheelSets{
            WheelSet along_y;
            WheelSet along_x;
        } wheel_sets; //saves all the set of wheels
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<carriage_control::carriageAction> ac;
        const std::string action_name;
        const std::string robot_name;
        const std::string base_name;
        //navigation controller 
        StraightNavigator *nav_manager_;
        carriage_control::carriageFeedback feedback_;
        carriage_control::carriageResult result_;
        long seconds; //amount of seconds that took robot to reach goal
        //client to obtain position of robot from gazebo topics
        ros::ServiceClient model_states_client;
        //gazebo server used to get position of model
        gazebo_msgs::GetModelState gazebo_srv;
        void getModelPosition();
        void getCell();
        void initializeVars();
        void registerCallbacks();
        void goalCB();
        //fills cell_stack to build a plan to the goal cell from start cell
        void orderCells(Cell start, Cell goal);
        void setWheelsUp(WheelSet& wheel_set);
        void setWheelsDown(WheelSet& wheel_set);
        
        //prepares robot to move in the direction of wheel_set
        void prepareRobot(WheelSet& wheel_set);
        //get accurate coordinates of cell
        Position getCellCenter(Cell cell);
        //return current coordinates via geometry_msgs::Pose
        geometry_msgs::Pose getPose();
        //fixate robot to current cell physically
        void fixRobot();
        //check if the goal is the same to the curr pos
        bool checkGoal(Cell c);
        bool executeDemoTasks(carriage_control::carriageGoalConstPtr goal);
        //cleans cells stack
        void cleanStack();
        //orders a circle returning to starting position
        void orderCircleCells();
};
};

#endif
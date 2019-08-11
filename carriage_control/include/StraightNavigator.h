#ifndef STRAIGHTNAVIGATOR_H_
#define STRAIGHTNAVIGATOR_H_


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <math.h>
class PoseGetter{
    public:
        virtual geometry_msgs::Pose getCurrentPose() = 0;
};
class Interrupt{
    public:
        virtual bool isInterruptCalled() = 0;
        //provides information of where should robot go upon receiving interrupts
        virtual void setInterruptExecuted() = 0;
        virtual geometry_msgs::PoseStamped getInterruptInfo() = 0;
};

class StraightNavigator{
    private:
        bool was_interrupt_already_called_ = false;
        geometry_msgs::Pose current;
        std::vector<nav_msgs::Odometry> vec_of_states_;
        //-------_!!!!!!!!!!!!_-------стремная проблема может вознукнуть с path_
        nav_msgs::Path path_;
        ros::Publisher * command_publisher_;
        char axis;
        PoseGetter* pose_getter_; //pointer to a function that get current position
        Interrupt* interruptor_; //helps us understand if interrupt was requested
        double a_max_; //max acceleration;
        double v_cruise_; //max velocity;
        double short_distance_; //if distance between poses is less/equal to this distance
                                //then short-distance strategy will be used
        double frequency_; //50 hz is minimum for good trajectory sampling
        double dt_; //sampling time
        void build_triangle_traj(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end,
            std::vector<nav_msgs::Odometry> &vec_of_states);
        void build_trapezoidal_traj(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end,
            std::vector<nav_msgs::Odometry> &vec_of_states);
        void calculateShortDistance();
        //gives 1 in case of long distance, 0 in case of short distance
        bool verifyLongDistance();
        void updatePose();
        void cleanPath();
        void interruptCB();
        geometry_msgs::PoseStamped getEndPose();
    public:
        void setInterrupt(Interrupt* interruptor);
        void setFrequency(double freq);
        void setSamplingTime(double t);
        StraightNavigator(double frequency, double max_vel, double a_max);
        ~StraightNavigator();
        void navigate();
        void build_traj();
        //pushes as first pose in path_
        void setStartPose(geometry_msgs::PoseStamped start_pose);
        //pushes as second pose in path_
        void setEndPose(geometry_msgs::PoseStamped end_pose);
        void setCommandPublisher(ros::Publisher* command_publisher);
        void setCurrentPoseGetter(PoseGetter* pose_getter);
        void setAxis(char axis);
        void centralize();
        
};
#endif
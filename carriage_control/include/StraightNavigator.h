#ifndef STRAIGHTNAVIGATOR_H_
#define STRAIGHTNAVIGATOR_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <math.h>

class StraightNavigator{
    private:
        std::vector<nav_msgs::Odometry> vec_of_states_;
        //-------_!!!!!!!!!!!!_-------стремная проблема может вознукнуть с path_
        nav_msgs::Path path_;
        std::string command_topic_;
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
    public:
        void setFrequency(double freq);
        void setSamplingTime(double t);
        StraightNavigator(std::string command_topic, double frequency, double max_vel, double a_max);
        ~StraightNavigator();
        void navigate();
        void build_traj();
        //pushes as first pose in path_
        void setStartPose(geometry_msgs::PoseStamped &start_pose);
        //pushes as second pose in path_
        void setEndPose(geometry_msgs::PoseStamped &end_pose);
};


#endif


#include <StraightNavigator.h>

void StraightNavigator::setFrequency(double freq) {frequency_ = freq; dt_ = 1/frequency_;}
void StraightNavigator::setSamplingTime(double dt) {dt_ = dt; frequency_ = 1/dt_;}

StraightNavigator::StraightNavigator(std::string command_topic, double freq, double max_vel, double a_max){
    setFrequency(freq);
    command_topic_ = command_topic;
    v_cruise_ = max_vel;
    a_max_ = a_max;
    //calculating criterion by which we tell what to use: triangular profile or trapezoidal one
    calculateShortDistance();
    //reserving some space in vector of poses in advance
    path_.poses.reserve(2);
}
void StraightNavigator::calculateShortDistance(){
    short_distance_ = 2 * 0.5 * pow(v_cruise_, 2) / a_max_; //we have maximum accelerations for 
        //ramp-up and ramp-down identical, so we multiply by two;
}

void StraightNavigator::setStartPose(geometry_msgs::PoseStamped &start_pose){
    path_.poses[0] = start_pose;
}
void StraightNavigator::setEndPose(geometry_msgs::PoseStamped &end_pose){
    path_.poses[1] = end_pose;
}

void StraightNavigator::build_triangle_traj(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end,
            std::vector<nav_msgs::Odometry> &vec_of_states){
    double x_start = start.pose.position.x;
    double x_end = end.pose.position.x;
    double y_start = start.pose.position.y;
    double y_end = end.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double distance = sqrt(dx*dx + dy*dy);
    double t_ramp = sqrt(distance / a_max_);
    double v_peak = t_ramp * a_max_;
    int ramp_res = ceil(t_ramp/dt_);
    nav_msgs::Odometry state;
    state.header = start.header;
    state.pose.pose = start.pose;
    //we are not modifying twist because robot is considered to be stopped at the beginning
    state.twist.twist.angular.x = 0;state.twist.twist.angular.y = 0;state.twist.twist.angular.z = 0;
    state.twist.twist.linear.x=0;state.twist.twist.linear.y=0;state.twist.twist.linear.z=0;
    double x_des = x_start;
    double y_des = y_start;
    double speed_des = 0.0;
    

}
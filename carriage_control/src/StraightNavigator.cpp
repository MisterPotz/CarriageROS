#include <StraightNavigator.h>

void StraightNavigator::setFrequency(double freq) {frequency_ = freq; dt_ = 1/frequency_;}
void StraightNavigator::setSamplingTime(double dt) {dt_ = dt; frequency_ = 1/dt_;}

StraightNavigator::StraightNavigator(double freq, double max_vel, double a_max){
    setFrequency(freq);
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
    short y_sign = 1;
    short x_sign = 1;
    short vel_sign = 1;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    if (dx <0 ) x_sign *=-1;
    if (dy <0) y_sign *=-1;
    //because carriage robot can move without reoorienting its pose, the directin in which it rides
    //should be understood. 0.01 here is error
    if (abs(dy) < 0.1){
        vel_sign = x_sign;
    } else
    {
        vel_sign = y_sign;
    }
    double psi_des = atan2(dy,dx);
    double distance = sqrt(dx*dx + dy*dy);
    double t_ramp = sqrt(distance / a_max_);
    double v_peak = t_ramp * a_max_;
    //how many points in one trajectory will be
    int ramp_res = ceil(t_ramp/dt_);
    nav_msgs::Odometry state;
    state.header = start.header;
    //orientation of the robot in the end is the same as in the start
    state.pose.pose = start.pose;
    //we are not modifying twist because robot is considered to be stopped at the beginning
    state.twist.twist.angular.x = 0;state.twist.twist.angular.y = 0;state.twist.twist.angular.z = 0;
    state.twist.twist.linear.x=0;state.twist.twist.linear.y=0;state.twist.twist.linear.z=0;
    
    double x_des = x_start;
    double y_des = y_start;
    vec_of_states.push_back(state);
    double des_vel = 0.0; 
    double t = 0.0;
    //ramp-up part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = t * a_max_*vel_sign;
        state.twist.twist.linear.x = des_vel;
        x_des = x_start + 0.5 * a_max_ * pow(t, 2) * cos(psi_des)*x_sign;
        y_des = y_start + 0.5 * a_max_ * pow(t,2) * sin(psi_des)*y_sign;
        state.pose.pose.position.x = x_des;
        state.pose.pose.position.y = y_des;
        vec_of_states.push_back(state);
    }
    //ramp-down part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = des_vel - a_max_*(t - t_ramp)*vel_sign;
        x_des = distance - 0.5 * a_max_ * pow(2*t_ramp - t, 2) * cos(psi_des)*x_sign;
        y_des = distance - 0.5 * a_max_ * pow(2*t_ramp - t, 2) * sin(psi_des)*y_sign;
        state.twist.twist.linear.x = des_vel;
        state.pose.pose.position.x = x_des;
        state.pose.pose.position.y = y_des;
        vec_of_states.push_back(state);
    }
    //now we should push the end state
    state.pose.pose = end.pose;
    //full stop
    state.twist.twist.linear.x = 0;
    vec_of_states.push_back(state);
}


void StraightNavigator::build_trapezoidal_traj(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end,
            std::vector<nav_msgs::Odometry> &vec_of_states){
    double x_start = start.pose.position.x;
    double x_end = end.pose.position.x;
    double y_start = start.pose.position.y;
    double y_end = end.pose.position.y;
    short y_sign = 1;
    short x_sign = 1;
    short vel_sign = 1;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    if (dx <0 ) x_sign *=-1;
    if (dy <0) y_sign *=-1;
    //because carriage robot can move without reoorienting its pose, the directin in which it rides
    //should be understood. 0.01 here is error
    if (abs(dy) < 0.1){
        vel_sign = x_sign;
    } else
    {
        vel_sign = y_sign;
    }
    double psi_des = atan2(dy,dx);
    double distance = sqrt(dx*dx + dy*dy);
    //in trapezoidal logic we find ramp time according to the v_cruise_ which is max
    double t_ramp = v_cruise_ / a_max_;
    double distance_ramp = a_max_ * t_ramp * t_ramp / 2;
    double v_peak = t_ramp * a_max_;
    //how many points in one trajectory will be
    int ramp_res = ceil(t_ramp/dt_);
    nav_msgs::Odometry state;
    state.header = start.header;
    //orientation of the robot in the end is the same as in the start
    state.pose.pose = start.pose;
    //we are not modifying twist because robot is considered to be stopped at the beginning
    state.twist.twist.angular.x = 0;state.twist.twist.angular.y = 0;state.twist.twist.angular.z = 0;
    state.twist.twist.linear.x=0;state.twist.twist.linear.y=0;state.twist.twist.linear.z=0;
    
    double x_des = x_start;
    double y_des = y_start;
    vec_of_states.push_back(state);
    double des_vel = 0.0; 
    double t = 0.0;
    //ramp-up part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = t * a_max_*vel_sign;
        state.twist.twist.linear.x = des_vel;
        x_des = x_start + 0.5 * a_max_ * pow(t, 2) * cos(psi_des)*x_sign;
        y_des = y_start + 0.5 * a_max_ * pow(t,2) * sin(psi_des)*y_sign;
        state.pose.pose.position.x = x_des;
        state.pose.pose.position.y = y_des;
        vec_of_states.push_back(state);
    }
    //preparing for straight- velocity profile part
    double distance_cruise = distance - 2*distance_ramp;
    double t_cruise = distance_cruise / v_cruise_;
    int cruise_res = ceil(t_cruise / dt_);
    //middle-part, same velocity zero acceleration
    des_vel = v_cruise_*vel_sign;
    for (int i = 0; i < cruise_res; i++){
        t+=dt_;
        x_des = distance_ramp + v_cruise_ * (t-t_ramp) * cos(psi_des)*x_sign;
        y_des = distance_ramp + v_cruise_ * (t-t_ramp) * sin(psi_des)*y_sign;
        state.pose.pose.position.x = x_des;
        state.pose.pose.position.y = y_des;
        vec_of_states.push_back(state);
    } 
    //ramp-down part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = des_vel - a_max_*(t - t_ramp-t_cruise)*vel_sign;
        x_des = distance - 0.5 * a_max_ * pow(2*t_ramp+t_cruise - t, 2) * cos(psi_des)*x_sign;
        y_des = distance - 0.5 * a_max_ * pow(2*t_ramp+t_cruise - t, 2) * sin(psi_des)*y_sign;
        state.twist.twist.linear.x = des_vel;
        state.pose.pose.position.x = x_des;
        state.pose.pose.position.y = y_des;
        vec_of_states.push_back(state);
    }
    //now we should push the end state
    state.pose.pose = end.pose;
    //full stop
    state.twist.twist.linear.x = 0;
    vec_of_states.push_back(state);
}
void StraightNavigator::navigate(){
    int size = vec_of_states_.size();
    //for cmd_vel accepts simplified twist messages, we do a bit of filling the msg before sending
    geometry_msgs::Twist des_state;
    ros::Rate loop_rate(frequency_);
    ROS_INFO("Initiating driving protocols...");
    for (int i = 0; i < size; i++){
        des_state.linear.x = vec_of_states_[i].twist.twist.linear.x;
        (*command_publisher_).publish(des_state);
        //here can be another publisher for twist in case of more complex moving
        loop_rate.sleep();
    }
}
void StraightNavigator::build_traj(){
    vec_of_states_.clear();
    if (verifyLongDistance()){
        build_triangle_traj(path_.poses[0], path_.poses[1], vec_of_states_);
        ROS_INFO("Short distance mode trajectory builder activated");
    }
    else
    {
        build_trapezoidal_traj(path_.poses[0], path_.poses[1], vec_of_states_);
        ROS_INFO("Long distance mode trajectory builder activated");
    }
}
bool StraightNavigator::verifyLongDistance(){
    double x_start = path_.poses[0].pose.position.x;
    double x_end = path_.poses[0].pose.position.x;
    double y_start = path_.poses[1].pose.position.y;
    double y_end = path_.poses[1].pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
    if (distance <= short_distance_){
        return false;
    }
    else
    {
        return true;
    }
    
}

void StraightNavigator::setCommandPublisher(ros::Publisher * command_publisher){
    command_publisher_ = command_publisher;
}
StraightNavigator::~StraightNavigator(){}


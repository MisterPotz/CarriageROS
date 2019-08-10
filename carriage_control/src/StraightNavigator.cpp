#include <StraightNavigator.h>

void StraightNavigator::setFrequency(double freq) {frequency_ = freq; dt_ = 1/frequency_;
 ROS_INFO("\t Period dt: %f", dt_); ROS_INFO("\t Freq: %f", frequency_);}
void StraightNavigator::setSamplingTime(double dt) {dt_ = dt; frequency_ = 1/dt_;
ROS_INFO("\t Period dt: %f", dt_); ROS_INFO("\t Freq: %f", frequency_);}

StraightNavigator::StraightNavigator(double freq, double max_vel, double a_max)  
{
    setFrequency(freq);
    v_cruise_ = max_vel;
    ROS_INFO("Max velocity: %f", v_cruise_);
    a_max_ = a_max;
    ROS_INFO("Max acceleration: %f", a_max_);
    path_.poses = std::vector<geometry_msgs::PoseStamped>(2);
    vec_of_states_ = std::vector<nav_msgs::Odometry>(100000);
    //calculating criterion by which we tell what to use: triangular profile or trapezoidal one
    calculateShortDistance();
    //reserving some space in vector of poses in advance
}
void StraightNavigator::calculateShortDistance(){
    short_distance_ = 2 * 0.5 * pow(v_cruise_, 2) / a_max_; //we have maximum accelerations for 
        //ramp-up and ramp-down identical, so we multiply by two;
    ROS_INFO("Short distance: %f", short_distance_);
}

void StraightNavigator::setStartPose(geometry_msgs::PoseStamped start_pose){
    path_.poses[0] = start_pose;
    ROS_INFO("Start pose coordinates: x: %f, y: %f", start_pose.pose.position.x, start_pose.pose.position.y);

}
void StraightNavigator::setEndPose(geometry_msgs::PoseStamped end_pose){
    path_.poses[1] = end_pose;
    ROS_INFO("End pose coordinates: x: %f, y: %f", end_pose.pose.position.x, end_pose.pose.position.y);

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
    ROS_INFO("\tBuilding traj -- dx: %f, dy: %f", dx, dy);
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
    ROS_INFO("\t\tPsi b//n poses: %f", psi_des);
    double distance = sqrt(dx*dx + dy*dy);
    double t_ramp = sqrt(distance / a_max_);
    double v_peak = t_ramp * a_max_*vel_sign;
    //how many points in one trajectory will be
    int ramp_res = round(t_ramp/dt_);
    ROS_INFO("\t\t ramp res: %d", ramp_res);
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
    des_vel = v_peak;
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = v_peak - a_max_*(t - t_ramp)*vel_sign; //Euler one-step integration
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
    ROS_INFO("\t\tBuilding traj -- dx: %f, dy: %f", dx, dy);
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
    ROS_INFO("\t\tPsi b//n poses: %f", psi_des);
    double distance = sqrt(dx*dx + dy*dy);
    //in trapezoidal logic we find ramp time according to the v_cruise_ which is max
    double t_ramp = v_cruise_ / a_max_;
    double distance_ramp = a_max_ * t_ramp * t_ramp * 0.5;
    //how many points in one trajectory will be
    int ramp_res = round(t_ramp/dt_);
    ROS_INFO("\t\t ramp res: %d", ramp_res);
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
    int cruise_res = round(t_cruise / dt_);
    ROS_INFO("\t\t cruise res: %d", ramp_res);
    //middle-part, same velocity zero acceleration
    des_vel = v_cruise_*vel_sign;
    for (int i = 0; i < cruise_res; i++){
        t+=dt_;
        x_des = distance_ramp + v_cruise_ * (t-t_ramp) * cos(psi_des)*x_sign;
        y_des = distance_ramp + v_cruise_ * (t-t_ramp) * sin(psi_des)*y_sign;
        state.pose.pose.position.x = x_des;
        state.pose.pose.position.y = y_des;
        state.twist.twist.linear.x = v_cruise_;
        vec_of_states.push_back(state);
    } 
    double time_part2of3 = t_ramp+ t_cruise;
    double total_time = time_part2of3 + t_ramp;
    //ramp-down part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = v_cruise_ - a_max_*(t - time_part2of3)*vel_sign;
        x_des = distance - 0.5 * a_max_ * pow(total_time - t, 2) * cos(psi_des)*x_sign;
        y_des = distance - 0.5 * a_max_ * pow(total_time - t, 2) * sin(psi_des)*y_sign;
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
    ROS_INFO("\t Size of vec_of_states: %d", size);
    //for cmd_vel accepts simplified twist messages, we do a bit of filling the msg before sending
    geometry_msgs::Twist des_state;
    ros::Rate loop_rate(frequency_);
    ROS_INFO("\tStarting action. Pub freq: %f", frequency_);
    for (int i = 0; i < size; i++){
        des_state.linear.x = vec_of_states_[i].twist.twist.linear.x;
        (*command_publisher_).publish(des_state);
        //here can be another publisher for twist in case of more complex moving
        loop_rate.sleep();
    }
    updatePose();
    ROS_INFO("Current pose: x:%f, y:%f", current.position.x, current.position.y);
    //ROS_INFO("\t Trajectory finished. Initiating centralizing.");
    //centralize(path_.poses[1].pose);
}

void StraightNavigator::build_traj(){
    vec_of_states_.clear();
    if (verifyLongDistance()){
        ROS_INFO("\tLong distance mode trajectory builder activated");
        build_trapezoidal_traj(path_.poses[0], path_.poses[1], vec_of_states_);
    }
    else
    {
        ROS_INFO("\tShort distance mode trajectory builder activated");
        build_triangle_traj(path_.poses[0], path_.poses[1], vec_of_states_);
    }
}
bool StraightNavigator::verifyLongDistance(){
    double x_start = path_.poses[0].pose.position.x;
    double x_end = path_.poses[1].pose.position.x;
    double y_start = path_.poses[0].pose.position.y;
    double y_end = path_.poses[1].pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    ROS_INFO("\tVerifying distance -- dx: %f, dy: %f", dx, dy);
    double distance = sqrt(dx*dx+dy*dy);
    ROS_INFO("\tDistance b/n poses: %f", distance);
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

void StraightNavigator::setCurrentPoseGetter(PoseGetter* pose_getter){
    pose_getter_ = pose_getter;
}
void StraightNavigator::updatePose(){
    geometry_msgs::Pose temp = pose_getter_->getCurrentPose();
    current.position.x = temp.position.x;
    current.position.y = temp.position.y;
    current.orientation = temp.orientation;
}
void StraightNavigator::centralize(geometry_msgs::Pose goal){
    //getting current pose
    updatePose();
    double  *curr_x, *curr_y;
    double goal_x, goal_y, dx, dy;
    curr_x = &current.position.x;
    curr_y = &current.position.y;
    goal_x = goal.position.x;
    goal_y = goal.position.y;
    dx = goal_x - *curr_x;
    dy = goal_y - *curr_y;
    double current_err = sqrt(dx*dx + dy*dy);
    double error = 0.0005; //allowed deviation
    //now let's analyze where is the bigger error
    double *current_watched_value, *goal_watched_value;
    if (0){ //abs(dx) > abs(dy)
        current_watched_value = curr_x;
        goal_watched_value = &goal_x;
        ROS_INFO("\t\tcentralizing along x");
    } else
    {
        current_watched_value = curr_y;
        goal_watched_value = &goal_y;
        ROS_INFO("\t\tcentralizing along y");
    }
    double des_speed;
    double sleep_freq = 2; //4 times in a second, 250 ms
    double speed_sign = 1.0; //tells in which direction to ride
    double difference = -*current_watched_value + *goal_watched_value;
    current_err = abs(difference); 
    ros::Rate sleeper(sleep_freq);
    ROS_INFO("\t\t Current pos: %f, goal pos: %f, difference: %f, given error: %f", *current_watched_value, *goal_watched_value, current_err, error);
    
    while(current_err >= error){
        //understanging in which direction to move
        // if (difference > 0){
        //     speed_sign = 1;
        // }
        // else{
        //     speed_sign = -1;
        // }
        // //finding speed
        // des_speed = abs(difference) * sleep_freq * speed_sign;
        // geometry_msgs::Twist msg;
        // msg.linear.x = des_speed;
        // //publishing message to move
        // (*command_publisher_).publish(msg);
        // //sleeping
        // sleeper.sleep();
        // msg.linear.x = 0;
        // //stopping
        // (*command_publisher_).publish(msg);
        //updating pose
        //build_triangle_traj()
        updatePose();
        //re-finding difference
        difference = -*current_watched_value + *goal_watched_value;
        //finding error
        current_err = abs(difference); 
        ROS_INFO("\t\t Current pos: %f, goal pos: %f, difference: %f, given error: %f", *current_watched_value, *goal_watched_value, current_err, error);

    }
}
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
    double pos_start = start.pose.position.x;
    double pos_end = end.pose.position.x;
    switch (axis){
        case 'x': pos_start = start.pose.position.x; pos_end=end.pose.position.x; break;
        case 'y': pos_start = start.pose.position.y; pos_end=end.pose.position.y; break;
    }
    double d_pos = pos_end - pos_start;
    double sign = (d_pos > 0.0) ? 1.0 : -1.0;
    ROS_INFO("\t\tBuilding traj -- d_distance: %f", d_pos);
    double t_ramp = sqrt(abs(d_pos) / a_max_);
    double v_peak = t_ramp * a_max_*sign;
    //how many points in one trajectory will be
    int ramp_res = round(t_ramp/dt_);
    ROS_INFO("\t\t ramp res: %d", ramp_res);
    nav_msgs::Odometry state;
    nav_msgs::Odometry zero_state;
    zero_state.twist.twist.angular.x = 0;zero_state.twist.twist.angular.y = 0;zero_state.twist.twist.angular.z = 0;
    zero_state.twist.twist.linear.x=0;zero_state.twist.twist.linear.y=0;zero_state.twist.twist.linear.z=0;
    state.header = start.header;
    state.pose = zero_state.pose;
    state.twist = zero_state.twist;
    //orientation of the robot in the end is the same as in the start
    state.pose.pose = start.pose;

    double des_pos = pos_start;
    vec_of_states.push_back(state);
    double des_vel = 0.0; 
    double t = 0.0;
    //ramp-up part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = t * a_max_*sign;
        state.twist.twist.linear.x = des_vel;
        state.pose = zero_state.pose;
        if (axis == 'x') state.pose.pose.position.x = des_pos + 0.5 * a_max_ * pow(t, 2)*sign;
        else state.pose.pose.position.y = des_pos + 0.5 * a_max_ * pow(t, 2)*sign;
        vec_of_states.push_back(state);
    }
    //ramp-down part
    des_vel = v_peak;
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = v_peak - a_max_*(t - t_ramp)*sign; //Euler one-step integration
        state.twist.twist.linear.x = des_vel;
        state.pose = zero_state.pose;
        if (axis == 'x') state.pose.pose.position.x = d_pos - 0.5 * a_max_ * pow(2*t_ramp - t, 2) *sign;
        else state.pose.pose.position.y = d_pos - 0.5 * a_max_ * pow(2*t_ramp - t, 2) *sign;
        vec_of_states.push_back(state);
    }
    state.pose = zero_state.pose;
    state.twist = zero_state.twist;
    //now we should push the end state
    state.pose.pose = end.pose;
    //full stop
    state.twist.twist.linear.x = 0;
    vec_of_states.push_back(state);
}


void StraightNavigator::build_trapezoidal_traj(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end,
            std::vector<nav_msgs::Odometry> &vec_of_states){
    double pos_start = start.pose.position.x;
    double pos_end = end.pose.position.x;
    switch (axis){
        case 'x': pos_start = start.pose.position.x; pos_end=end.pose.position.x; break;
        case 'y': pos_start = start.pose.position.y; pos_end=end.pose.position.y; break;
    }
    double d_pos = pos_end - pos_start;
    double sign = (d_pos > 0.0) ? 1.0 : -1.0;
    ROS_INFO("\t\tBuilding traj -- d_distance: %f", d_pos);
    //in trapezoidal logic we find ramp time according to the v_cruise_ which is max
    double t_ramp = v_cruise_ / a_max_;
    double distance_ramp = a_max_ * t_ramp * t_ramp * 0.5;
    //how many points in one trajectory will be
    int ramp_res = round(t_ramp/dt_);
    ROS_INFO("\t\t ramp res: %d", ramp_res);
    nav_msgs::Odometry state;
    nav_msgs::Odometry zero_state;
    zero_state.twist.twist.angular.x = 0;zero_state.twist.twist.angular.y = 0;zero_state.twist.twist.angular.z = 0;
    zero_state.twist.twist.linear.x=0;zero_state.twist.twist.linear.y=0;zero_state.twist.twist.linear.z=0;
    state.header = start.header;
    state.pose = zero_state.pose;
    state.twist = zero_state.twist;
    //orientation of the robot in the end is the same as in the start
    state.pose.pose = start.pose;
    //we are not modifying twist because robot is considered to be stopped at the beginning
    
    double des_pos = pos_start;
    vec_of_states.push_back(state);
    double des_vel = 0.0; 
    double t = 0.0;
    //ramp-up part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = t * a_max_*sign;
        state.twist.twist.linear.x = des_vel;
        state.pose = zero_state.pose;
        if (axis == 'x') state.pose.pose.position.x = des_pos + 0.5 * a_max_ * pow(t, 2)*sign;
        else state.pose.pose.position.y = des_pos + 0.5 * a_max_ * pow(t, 2)*sign;
        vec_of_states.push_back(state);
    }
    //preparing for straight- velocity profile part
    double distance_cruise = abs(d_pos) - 2*distance_ramp;
    double t_cruise = distance_cruise / v_cruise_;
    int cruise_res = round(t_cruise / dt_);
    ROS_INFO("\t\t cruise res: %d", ramp_res);
    //middle-part, same velocity zero acceleration
    des_vel = v_cruise_*sign;
    for (int i = 0; i < cruise_res; i++){
        t+=dt_;
        state.pose = zero_state.pose;
        if (axis == 'x') state.pose.pose.position.x = distance_ramp + v_cruise_ * (t-t_ramp)*sign;
        else state.pose.pose.position.y = distance_ramp + v_cruise_ * (t-t_ramp)*sign;
        state.twist.twist.linear.x = des_vel;
        vec_of_states.push_back(state);
    } 
    double time_part2of3 = t_ramp+ t_cruise;
    double total_time = time_part2of3 + t_ramp;
    //ramp-down part
    for (int i =0; i < ramp_res; i++){
        t+=dt_;
        des_vel = v_cruise_*sign - a_max_*(t - time_part2of3)*sign;
        state.twist.twist.linear.x = des_vel;
        state.pose = zero_state.pose;
        if (axis == 'x') state.pose.pose.position.x = d_pos - 0.5 * a_max_ * pow(total_time - t, 2) *sign;
        else state.pose.pose.position.y = d_pos - 0.5 * a_max_ * pow(total_time - t, 2) *sign;
        vec_of_states.push_back(state);
    }
    state.pose = zero_state.pose;
    state.twist = zero_state.twist;
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
void StraightNavigator::centralize(){
    //getting current pose
    updatePose();
    double *current_value;
    double goal_point;
    geometry_msgs::Pose saved_goal = getEndPose().pose;
    //selecting values to fix according to the current chosen axis
    switch (axis){
        case 'x': current_value = &current.position.x; goal_point=saved_goal.position.x; break;
        case 'y': current_value = &current.position.y; goal_point=saved_goal.position.y; break;
        default: ROS_INFO("\t\t\tSome really strange thins occured in StraightNavigator::centralize()"); return;
    }
    double current_err = goal_point - *current_value;
    double error = 0.002; //allowed deviation
    double sleeping_time = 0.1;
    ros::Duration wait(sleeping_time);
    double speed_sign = (current_err >0) ? 1 : -1;
    double speed = abs(current_err)/ sleeping_time * speed_sign;
    ROS_INFO("\t\t Current pos: %f, goal pos: %f, difference: %f, given error: %f", *current_value, goal_point, current_err, error); 
    do{
        // cleanPath();
        // geometry_msgs::PoseStamped pose;
        // //setting starting pose
        // pose.pose = current;
        // setStartPose(pose);
        // //setting ending pose
        // pose.pose = saved_goal;
        // setEndPose(pose);
        //build_traj();
        //navigate();
        geometry_msgs::Twist msg;
        msg.linear.x = speed;
        command_publisher_->publish(msg);
        wait.sleep();
        msg.linear.x = 0;
        command_publisher_->publish(msg);
        updatePose();
        current_err = goal_point - *current_value;
        speed_sign = (current_err >0) ? 1 : -1;
        speed = abs(current_err)/ sleeping_time * speed_sign;
        ROS_INFO("\t\t Current pos: %f, goal pos: %f, difference: %f, given error: %f", *current_value, goal_point, current_err, error); 
    }while(abs(goal_point - *current_value) >= error );

}

void StraightNavigator::cleanPath(){
    path_.poses.clear();
}
void StraightNavigator::setAxis(char ch){
    axis = ch;
}
geometry_msgs::PoseStamped StraightNavigator::getEndPose(){
    return path_.poses[1];
}
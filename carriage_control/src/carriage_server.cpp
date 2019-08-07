#include <carriage_server.h>

carriage_control::Carriage_Server::Carriage_Server(std::string name, std::string _robot_name, float _cell_size) :
    ac(nh, name, false),
    action_name(name), robot_name(_robot_name), cell_size(_cell_size)
  {
    initializeVars();
    registerCallbacks();
    ac.start();
  }
void carriage_control::Carriage_Server::initializeVars(){
    gazebo_srv.request.model_name = robot_name;
    model_states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
}

void carriage_control::Carriage_Server::registerCallbacks(){
  ac.registerGoalCallback(boost::bind(&carriage_control::Carriage_Server::goalCB, this));
}
void carriage_control::Carriage_Server::getCell(){
    double temp_x = position.x / cell_size;
    double temp_y = position.y / cell_size;
    cell.x = (temp_x > 0) ?  trunc(temp_x)+1 : trunc(temp_x)-1;
    cell.y = (temp_y > 0) ?  trunc(temp_y)+1 : trunc(temp_y)-1;
}

carriage_control::Carriage_Server::~Carriage_Server(){}
void carriage_control::Carriage_Server::getModelPosition() 
{
    if (model_states_client.call(gazebo_srv))
        if (gazebo_srv.response.success){
            position.x = gazebo_srv.response.pose.position.x;
            position.y = gazebo_srv.response.pose.position.y;
        }
        else throw std::invalid_argument("Can not retrieve robot position");
}
void carriage_control::Carriage_Server::showCell(){
    try{
        carriage_control::Carriage_Server::getModelPosition();
        carriage_control::Carriage_Server::getCell();
    } catch (std::invalid_argument& e){
        ROS_ERROR(e.what());
    }
    ROS_INFO("Current cell:\nx:%d, y:%d", cell.x, cell.y);
}
const carriage_control::Carriage_Server::Cell carriage_control::Carriage_Server::orderCells( carriage_control::Carriage_Server::Cell& goal){
  Cell trajectory;
  //update current position
  carriage_control::Carriage_Server::getCell();
  trajectory.x = goal.x - cell.x;
  trajectory.y = goal.y - cell.y;
  return trajectory;
}
void carriage_control::Carriage_Server::registerWheelSets(const WheelSet& x,const WheelSet& y){
  wheel_sets.along_x = x;
  wheel_sets.along_y = y;
}
bool carriage_control::Carriage_Server::moveRobot(const carriage_control::Carriage_Server::Cell trajectory){
  //first, we go along x
  try{
    if (trajectory.x > 0){
      setWheelsDown(wheel_sets.along_x);
      setWheelsUp(wheel_sets.along_y);
      spinWheels(wheel_sets.along_x, trajectory.x);
      centralize();
    }
    if (trajectory.y > 0){
      setWheelsDown(wheel_sets.along_y);
      setWheelsUp(wheel_sets.along_x);
      spinWheels(wheel_sets.along_y, trajectory.y);
      centralize();
    }
  } catch (std::runtime_error &e){
    //if something goes wrong we return false result representing problems
    return false;
  }
  ROS_INFO("Robot arrived");
  //if everything okay, we bring true to outer scope
  return true;
}
void carriage_control::Carriage_Server::goalCB(){
  if (!ac.isActive()){
    return;
  }
  carriage_control::carriageGoalConstPtr goal =  ac.acceptNewGoal();
  Cell goal_c; goal_c.x = goal->x_cell; goal_c.y = goal->y_cell;
  if(moveRobot(orderCells(goal_c))){
    result_.success = true;
    result_.used_time = seconds;
    ac.setSucceeded(result_, "Robot successfully arrived");
  }
  else{
    result_.success = false;
    result_.used_time = seconds;
    ac.setAborted(result_, "Some error occurred");
  }
 
}
//this is high-level function to build trajectory
void carriage_control::Carriage_Server::spinWheels(const WheelSet& wheel_set, int cells){
  //set paths (start - current pos, end - the middle of the next cell)
  //give paths to path-filter function to build comprehensible states of robot, also we give a reference to vector of Odometry msgs
  //publish received states to cmd_vel
}
//this function centralizes robot directly into middle of cell, returnes status of completion
bool carriage_control::Carriage_Server::centralize(){

}
void carriage_control::Carriage_Server::setWheelsUp(WheelSet& wheel_set){
  std_msgs::Float64 msg;
  msg.data = upper_position;
  for (int i =0; i < 4; i++){
    wheel_set.drive_joint_command_pub[i].publish(msg);
  }
}
void carriage_control::Carriage_Server::setWheelsDown(WheelSet& wheel_set){
  std_msgs::Float64 msg;
  msg.data = lower_position;
  for (int i =0; i < 4; i++){
    wheel_set.drive_joint_command_pub[i].publish(msg);
  }
}
double carriage_control::Carriage_Server::getCellCize(){
  return cell_size;
}
ros::NodeHandle& carriage_control::Carriage_Server::getNodeHandle(){
  return nh;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "carriage_server");
  carriage_control::Carriage_Server crrg_srvr(ros::this_node::getName(), "bot", 0.78);
  ros::NodeHandle& nh =  crrg_srvr.getNodeHandle();
  carriage_control::WheelSet along_y;
  along_y.cmd_vel="/cmd_vel_y";
  along_y.drive_joint_command_pub[0] = nh.advertise<std_msgs::Float64>("/bot/1y_position_controller/command", 5); 
  along_y.drive_joint_command_pub[1] = nh.advertise<std_msgs::Float64>("/bot/2y_position_controller/command", 5);
  along_y.drive_joint_command_pub[2] = nh.advertise<std_msgs::Float64>("/bot/3y_position_controller/command", 5);
  along_y.drive_joint_command_pub[3] = nh.advertise<std_msgs::Float64>("/bot/4y_position_controller/command", 5);
  carriage_control::WheelSet along_x;
  along_x.cmd_vel="/cmd_vel_x";
  along_x.drive_joint_command_pub[0] = nh.advertise<std_msgs::Float64>("/bot/1x_position_controller/command", 5); 
  along_x.drive_joint_command_pub[1] = nh.advertise<std_msgs::Float64>("/bot/2x_position_controller/command", 5);
  along_x.drive_joint_command_pub[2] = nh.advertise<std_msgs::Float64>("/bot/3x_position_controller/command", 5);
  along_x.drive_joint_command_pub[3] = nh.advertise<std_msgs::Float64>("/bot/4x_position_controller/command", 5);
  crrg_srvr.registerWheelSets(along_x, along_y);
  ros::Rate rate(4);
  
  while(ros::ok()){
    ros::spinOnce();
    crrg_srvr.showCell();
    rate.sleep();
  }
    return 0;
}
#include <carriage_server.h>

carriage_control::Carriage_Server::Carriage_Server(std::string name, std::string _robot_name) :
    ac(nh, name, false),
    action_name(name), robot_name(_robot_name)
  {
    initializeVars();
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
    cell.x = (position.x > 0) ?  trunc(position.x)+1 : trunc(position.x)-1;
    cell.y = (position.y > 0) ?  trunc(position.y)+1 : trunc(position.y)-1;
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
const carriage_control::Carriage_Server::Cell carriage_control::Carriage_Server::buildTrajectoryToGoal( Cell& goal){
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
bool carriage_control::Carriage_Server::applyTrajectory(const carriage_control::Carriage_Server::Cell trajectory){
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
  if(applyTrajectory(buildTrajectoryToGoal(goal_c))){
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
ros::NodeHandle& carriage_control::Carriage_Server::getNodeHandle(){
  return nh;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "carriage_server");
  carriage_control::Carriage_Server crrg_srvr(ros::this_node::getName(), "bot");
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
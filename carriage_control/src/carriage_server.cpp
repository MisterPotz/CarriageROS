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
  ac.registerGoalCallback(boost::bind(&carriage_control::Carriage_Server::goalCB, this, _1));
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
const carriage_control::Carriage_Server::Cell carriage_control::Carriage_Server::buildTrajectoryToGoal(const Cell& goal){
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

int main(int argc, char** argv){
  ros::init(argc, argv, "carriage_server");
  carriage_control::Carriage_Server crrg_srvr(ros::this_node::getName(), "bot");
  carriage_control::WheelSet along_y;
  along_y.cmd_vel="/cmd_vel_y";
  along_y.drive_joint_command[0] = "/bot/1y_position_controller/command"; 
  along_y.drive_joint_command[1] = "/bot/2y_position_controller/command";
  along_y.drive_joint_command[2] = "/bot/3y_position_controller/command";
  along_y.drive_joint_command[3] = "/bot/4y_position_controller/command";
  carriage_control::WheelSet along_x;
  along_x.cmd_vel="/cmd_vel_x";
  along_x.drive_joint_command[0] = "/bot/1x_position_controller/command"; 
  along_x.drive_joint_command[1] = "/bot/2x_position_controller/command";
  along_x.drive_joint_command[2] = "/bot/3x_position_controller/command";
  along_x.drive_joint_command[3] = "/bot/4x_position_controller/command";
  crrg_srvr.registerWheelSets(along_x, along_y);
  ros::Rate rate(4);
  while(ros::ok()){
    ros::spinOnce();
    crrg_srvr.showCell();
    rate.sleep();
  }
    return 0;
}
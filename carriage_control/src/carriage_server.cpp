#include <carriage_server.h>

Carriage_Server::Carriage_Server(std::string name, std::string _robot_name) :
    ac(nh, name, false),
    action_name(name), robot_name(_robot_name)
  {
    initializeVars();
    ac.start();
  }
void Carriage_Server::initializeVars(){
    gazebo_srv.request.model_name = robot_name;
    model_states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

}
void Carriage_Server::getCell(){
    cell.x = (position.x > 0) ?  trunc(position.x)+1 : trunc(position.x)-1;
    cell.y = (position.y > 0) ?  trunc(position.y)+1 : trunc(position.y)-1;
}

Carriage_Server::~Carriage_Server(){}
void Carriage_Server::getModelPosition() 
{
    if (model_states_client.call(gazebo_srv))
        if (gazebo_srv.response.success){
            position.x = gazebo_srv.response.pose.position.x;
            position.y = gazebo_srv.response.pose.position.y;
        }
        else throw std::invalid_argument("Can not retrieve robot position");
}
void Carriage_Server::showCell(){
    try{
        Carriage_Server::getModelPosition();
        Carriage_Server::getCell();
    } catch (std::invalid_argument& e){
        ROS_ERROR(e.what());
    }
    ROS_INFO("Current cell:\nx:%d, y:%d", cell.x, cell.y);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "carriage_server");
  Carriage_Server crrg_srvr(ros::this_node::getName(), "bot");
  ros::Rate rate(4);
  while(ros::ok()){
    ros::spinOnce();
    crrg_srvr.showCell();
    rate.sleep();
  }
    return 0;
}
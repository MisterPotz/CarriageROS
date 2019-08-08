#include <carriage_server.h>

carriage_control::Carriage_Server::Carriage_Server(std::string name, std::string _robot_name,
    std::string _base_name, float _cell_size) :
    ac(nh, name, false),
    action_name(name), robot_name(_robot_name), base_name(_base_name), cell_size(_cell_size)
  {
    nav_manager_ = new StraightNavigator(50, 1.0, 0.4);
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

carriage_control::Carriage_Server::Position carriage_control::Carriage_Server::getCellCenter(Cell cell){
  Position cell_center;
  cell_center.x = (cell.x > 0) ? (cell.x-1)*cell_size + cell_size * 0.5 : 
                                  (cell.x+1)*cell_size - cell_size * 0.5;
  cell_center.y = (cell.y > 0) ? (cell.y-1)*cell_size + cell_size * 0.5 : 
                                  (cell.y+1)*cell_size - cell_size * 0.5;
  return cell_center;
}

carriage_control::Carriage_Server::~Carriage_Server(){
  delete nav_manager_;
}
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
void carriage_control::Carriage_Server::orderCells( carriage_control::Carriage_Server::Cell goal){
  //cleaning stack
  while (!cell_order_.empty())
    cell_order_.pop();
  if (goal.x == cell.x && goal.y == cell.y){
    //if current cell is equal to goal cell
    return;
  }
  //update current position
  carriage_control::Carriage_Server::getCell();
  carriage_control::Carriage_Server::Cell middle;
  cell_order_.push(goal);
  middle.x = cell.x;
  middle.y = goal.y;
  cell_order_.push(middle);
}
void carriage_control::Carriage_Server::registerWheelSets(const WheelSet& x,const WheelSet& y){
  wheel_sets.along_x = x;
  wheel_sets.along_y = y;
}
bool carriage_control::Carriage_Server::checkOrder(){
  if (cell_order_.empty())
    return false;
  else
  {
    return true;
  }
}
void carriage_control::Carriage_Server::prepareRobot(WheelSet& wheel_set){
  setWheelsUp(wheel_sets.along_y);
  setWheelsUp(wheel_sets.along_x);
  setWheelsDown(wheel_set);
}
bool carriage_control::Carriage_Server::moveRobot2Cell(carriage_control::Carriage_Server::Cell goal){
  //first, we buid an order of visiting cells
  orderCells(goal);
  if (checkOrder()){
    while (!cell_order_.empty()){
      try{
        Cell end = cell_order_.top();
        cell_order_.pop();
        Cell start = cell;
        if (end.x - start.x == 0){
          prepareRobot(wheel_sets.along_y);
          nav_manager_->setCommandPublisher(&wheel_sets.along_y.twist_command_pub);
        }
        else {
          prepareRobot(wheel_sets.along_x);
          nav_manager_->setCommandPublisher(&wheel_sets.along_x.twist_command_pub);
        }
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id=base_name;
        Position start_pos = getCellCenter(start);
        Position end_pos = getCellCenter(end);
        pose_stamped.pose.position.x = start_pos.x;
        pose_stamped.pose.position.y = start_pos.y;
        nav_manager_->setStartPose(pose_stamped);
        pose_stamped.pose.position.x = end_pos.x;
        pose_stamped.pose.position.y = end_pos.y;
        nav_manager_->setEndPose(pose_stamped);
        nav_manager_->build_traj();
        nav_manager_->navigate();

      } catch (std::runtime_error &e){
        //if something goes wrong we return false result representing problems
        return false;
      }
    }
  }
  //first, we go along x
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
  if(moveRobot2Cell(goal_c)){
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

#include <carriage_server.h>

carriage_control::TimeControl::TimeControl(double sleep_rate) : sleep_rate_(sleep_rate){

}
carriage_control::TimeControl::~TimeControl(){

}
void carriage_control::TimeControl::wait(){
  sleep_rate_.sleep();
}
void carriage_control::Carriage_Server::setTimeControl(carriage_control::TimeControl* time_control){
  time_control_ = time_control;
}
carriage_control::Carriage_Server::Carriage_Server(std::string name, std::string _robot_name,
    std::string _base_name, float _cell_size) :
    ac(nh, name, false),
    action_name(name), robot_name(_robot_name), base_name(_base_name), cell_size(_cell_size)
  {
    nav_manager_ = new StraightNavigator(80, 1, 0.5);
    nav_manager_->setCurrentPoseGetter(this);
    initializeVars();
    registerCallbacks();
    cleanStack();
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
    getModelPosition();
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
    if (model_states_client.call(gazebo_srv)){
       if (gazebo_srv.response.success){
            position.x = gazebo_srv.response.pose.position.x;
            position.y = gazebo_srv.response.pose.position.y;
        }
    }
    else throw std::invalid_argument("Can not retrieve robot position");
}
void carriage_control::Carriage_Server::showCell(){
    try{
        carriage_control::Carriage_Server::getModelPosition();
        carriage_control::Carriage_Server::getCell();
    } catch (const std::invalid_argument& e){
        ROS_ERROR(e.what());
    }
    ROS_INFO("Current cell:\nx:%d, y:%d", cell.x, cell.y);
}
void carriage_control::Carriage_Server::orderCells( carriage_control::Carriage_Server::Cell start,carriage_control::Carriage_Server::Cell goal){
  //getCell();
  //cleaning stack
  if (goal.x == start.x && goal.y == start.y){
    //if current cell is equal to goal cell
    return;
  }
  if (goal.x == start.x || goal.y == start.y){
    cell_order_.push(goal);
    return;
  }
  //update current position
  carriage_control::Carriage_Server::Cell middle;
  cell_order_.push(goal);
  middle.x = start.x;
  middle.y = goal.y;
  cell_order_.push(middle);
}
void carriage_control::Carriage_Server::registerWheelSets(const WheelSet& x,const WheelSet& y){
  wheel_sets.along_x = x;
  wheel_sets.along_y = y;
}
void carriage_control::Carriage_Server::prepareRobot(WheelSet& wheel_set){
  setWheelsDown(wheel_sets.along_y);
  setWheelsDown(wheel_sets.along_x);
  //giving some time to robot to become prepared
  time_control_->wait();
  setWheelsUp(wheel_set);
  time_control_->wait();
}
void carriage_control::Carriage_Server::fixRobot(){
  setWheelsDown(wheel_sets.along_y);
  setWheelsDown(wheel_sets.along_x);
  //giving some time to robot to become prepared
  time_control_->wait();
}
bool carriage_control::Carriage_Server::executeDemoTasks(carriage_control::carriageGoalConstPtr goal){
  if (goal->demo_dropdown_wheels){
    setWheelsDown(wheel_sets.along_x);
    setWheelsDown(wheel_sets.along_y);
    time_control_->wait();
    return true;
  }
  if (goal->demo_lift_wheels){
    setWheelsUp(wheel_sets.along_x);
    setWheelsUp(wheel_sets.along_y);
    time_control_->wait();
    return true;
  }
  if (goal->demo_ride_circle){
    cleanStack();
    orderCircleCells();
    return false;
  }
  return false;
}
bool carriage_control::Carriage_Server::moveRobot2Cell(){
  //first, we buid an order of visiting cells
  try{
    while (!cell_order_.empty()){
        getCell();
        Cell end = cell_order_.top();
        cell_order_.pop();
        Cell start = cell;
        if (end.x - start.x == 0){
          prepareRobot(wheel_sets.along_x); //giving wheel set that should be set up
          nav_manager_->setCommandPublisher(&wheel_sets.along_y.twist_command_pub);
          nav_manager_->setAxis(wheel_sets.along_y.axis);
        }
        else {
          prepareRobot(wheel_sets.along_y);
          nav_manager_->setCommandPublisher(&wheel_sets.along_x.twist_command_pub);
          nav_manager_->setAxis(wheel_sets.along_x.axis);
        }
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id=base_name;
        getModelPosition();
        Position start_pos = position;
        Position end_pos = getCellCenter(end);
        pose_stamped.pose.position.x = start_pos.x;
        pose_stamped.pose.position.y = start_pos.y;
        nav_manager_->setStartPose(pose_stamped);
        pose_stamped.pose.position.x = end_pos.x;
        pose_stamped.pose.position.y = end_pos.y;
        nav_manager_->setEndPose(pose_stamped);
        nav_manager_->build_traj();
        nav_manager_->navigate();
        time_control_->wait();
        ROS_INFO("\t Trajectory finished. Initiating centralizing.");
        nav_manager_->centralize();
      } 
      //cleaning stack is unnecessary - it cleans itself during excecution
      //fixing robot in-place
      fixRobot();
      ROS_INFO("Robot arrived!");
      return true;
    }catch (const std::invalid_argument& e){
        ROS_INFO("%s", e.what());
    }catch (const std::runtime_error &e){
      //if something goes wrong we return false result representing problems
      ROS_INFO("%s", e.what());
      return false;
    }
  return false;
  //first, we go along x
  //if everything okay, we bring true to outer scope
}
bool carriage_control::Carriage_Server::checkGoal(Cell c){
  getCell();
  if (c.x == cell.x && c.y==cell.y){
    return false;
  }
  else
    return true;
}
void carriage_control::Carriage_Server::goalCB(){
  cleanStack();
  getCell();
  if (ac.isActive()){
    return;
  }
  carriage_control::carriageGoalConstPtr goal =  ac.acceptNewGoal();
  Cell goal_c; goal_c.x = goal->x_cell; goal_c.y = goal->y_cell;
  if (goal_c.x == 0 && goal_c.y ==0){
    goal_c.x = 1;
    goal_c.y = 1;
  }
  bool executeDemoTaskFlag = false;
  if (executeDemoTasks(goal)){
    //filling stack with destinations
    executeDemoTaskFlag = true;
    ROS_INFO("Demo action performed");
    ac.setSucceeded(result_,"Demo action performed");
  }
  //check goal function updates current position
  ROS_INFO("Accepting new goal, x: %d, y:%d", goal_c.x, goal_c.y);
  if (cell_order_.empty()){
    orderCells(cell, goal_c);
  }
  //moving robot to goal
  if(!executeDemoTaskFlag && moveRobot2Cell()){
    result_.success = true;
    result_.used_time = seconds;
    ac.setSucceeded(result_, "Robot successfully arrived");

  }
  else{
    if (executeDemoTaskFlag){
      return;
    }
    result_.success = false;
    result_.used_time = seconds;
    ac.setAborted(result_, "Some error occurred");
  }
 
}

void carriage_control::Carriage_Server::setWheelsUp(WheelSet& wheel_set){
  std_msgs::Float64 msg;
  msg.data = upper_position;
  wheel_set.drive_joint_command_pub[0].publish(msg);
  wheel_set.drive_joint_command_pub[2].publish(msg);
  wheel_set.drive_joint_command_pub[1].publish(msg);
  wheel_set.drive_joint_command_pub[3].publish(msg);
}
void carriage_control::Carriage_Server::setWheelsDown(WheelSet& wheel_set){
  std_msgs::Float64 msg;
  msg.data = lower_position;
  wheel_set.drive_joint_command_pub[0].publish(msg);
  wheel_set.drive_joint_command_pub[2].publish(msg);
  wheel_set.drive_joint_command_pub[1].publish(msg);
  wheel_set.drive_joint_command_pub[3].publish(msg);
}
double carriage_control::Carriage_Server::getCellCize(){
  return cell_size;
}
ros::NodeHandle& carriage_control::Carriage_Server::getNodeHandle(){
  return nh;
}

geometry_msgs::Pose carriage_control::Carriage_Server::getCurrentPose(){
  geometry_msgs::Pose pose;
  //getting position from gaziba
  getModelPosition();
  pose = gazebo_srv.response.pose;
  return pose;
}
void carriage_control::Carriage_Server::cleanStack(){
  while (!cell_order_.empty())
    cell_order_.pop();
}
void carriage_control::Carriage_Server::orderCircleCells(){
  Cell a; Cell b;
  getCell();
  a = cell;
  //b -opposite to a
  b.x = -a.x;
  b.y = -a.y;
  ROS_INFO("Opposite cell: x:%d, y:%d", b.x, b.y);
  //another. finalizing circle
  orderCells(b, a);
  //one direction
  orderCells(a,b);
  
}
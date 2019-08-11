#include <StraightNavigator.h>
#include <carriage_server.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "carriage_server");
  carriage_control::Carriage_Server crrg_srvr(ros::this_node::getName(), "bot","base_link", 0.75);
  //when we created crrg_srvr we instantiated and initialized nodehandle
  //now we may create timecontrol class which involves ros::Rate. it cannot be created inside a class
  //which itself creates nodehandle (ros::rate requires nodehandle to be created first)
  carriage_control::TimeControl time_control(1.0);//pass in frequency
  crrg_srvr.setTimeControl(&time_control);
  ros::NodeHandle& nh =  crrg_srvr.getNodeHandle();
  carriage_control::WheelSet along_y;
  along_y.twist_command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_y", 4);
  ros::Rate sleep(0.4);
  along_y.drive_joint_command_pub[0] = nh.advertise<std_msgs::Float64>("/bot/1y_position_controller/command", 5); 
  along_y.drive_joint_command_pub[1] = nh.advertise<std_msgs::Float64>("/bot/2y_position_controller/command", 5);
  along_y.drive_joint_command_pub[2] = nh.advertise<std_msgs::Float64>("/bot/3y_position_controller/command", 5);
  along_y.drive_joint_command_pub[3] = nh.advertise<std_msgs::Float64>("/bot/4y_position_controller/command", 5);
  along_y.axis = 'y';
  carriage_control::WheelSet along_x;
  along_x.twist_command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_x", 4);
  along_x.drive_joint_command_pub[0] = nh.advertise<std_msgs::Float64>("/bot/5x_position_controller/command", 5); 
  along_x.drive_joint_command_pub[1] = nh.advertise<std_msgs::Float64>("/bot/6x_position_controller/command", 5);
  along_x.drive_joint_command_pub[2] = nh.advertise<std_msgs::Float64>("/bot/7x_position_controller/command", 5);
  along_x.drive_joint_command_pub[3] = nh.advertise<std_msgs::Float64>("/bot/8x_position_controller/command", 5);
  along_x.axis = 'x';
  //waiting some time for topics to become registered within ROS
  sleep.sleep();
  crrg_srvr.registerWheelSets(along_x, along_y);
  ros::spin();
  return 0;
}
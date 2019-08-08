#include <StraightNavigator.h>
#include <carriage_server.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "carriage_server");
  carriage_control::Carriage_Server crrg_srvr(ros::this_node::getName(), "bot","base_link", 0.78);
  ros::NodeHandle& nh =  crrg_srvr.getNodeHandle();
  carriage_control::WheelSet along_y;
  along_y.twist_command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_y", 6);
  along_y.drive_joint_command_pub[0] = nh.advertise<std_msgs::Float64>("/bot/1y_position_controller/command", 5); 
  along_y.drive_joint_command_pub[1] = nh.advertise<std_msgs::Float64>("/bot/2y_position_controller/command", 5);
  along_y.drive_joint_command_pub[2] = nh.advertise<std_msgs::Float64>("/bot/3y_position_controller/command", 5);
  along_y.drive_joint_command_pub[3] = nh.advertise<std_msgs::Float64>("/bot/4y_position_controller/command", 5);
  carriage_control::WheelSet along_x;
  along_x.twist_command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_x", 6);
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
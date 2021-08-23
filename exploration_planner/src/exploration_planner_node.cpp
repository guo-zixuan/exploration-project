#include <ros/ros.h>

#include "exploration_planner/exploration_planner.hpp"

int main(int argc, char** argv){
  
  ros::init(argc, argv, "exploration_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  exploration_ns::exploration_planner ep(nh,nh_private);

  ros::spin();
  return 0;  
}
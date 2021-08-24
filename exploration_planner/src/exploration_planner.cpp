
#include "exploration_planner/exploration_planner.hpp"

exploration_ns::exploration_planner::exploration_planner(ros::NodeHandle& nh,ros::NodeHandle& nh_p)
:nh_(nh),
nh_private_(nh_p)
{
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);
  grid_ = new exploration_ns::grid(nh_,nh_private_);
  
  init();

}
exploration_ns::exploration_planner::~exploration_planner(){
  if (manager_) {
    delete manager_;
  }
  if (grid_) {
    delete grid_;
  }
}

bool exploration_ns::exploration_planner::init(){

    if(!this->setParams()){
      ROS_ERROR("Set parameters fail. Cannot start planning!");
    }

    odomSub_ = nh_.subscribe(odomSubTopic_, 10,
                  &exploration_ns::exploration_planner::odomCallback, this);
    mapSub_ = nh_.subscribe(mapSubTopic_, 10,
                  &exploration_ns::exploration_planner::mapCallback, this);

    odomPub_ = nh_.advertise<nav_msgs::Odometry>("/test_odom",100);

    mapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/test_map",100);

    return true;
}

bool exploration_ns::exploration_planner::setParams(){
  
  nh_private_.getParam("/planner/odomTopic", odomSubTopic_);
  nh_private_.getParam("/planner/gridMapTopic", mapSubTopic_);

  return true;

}

void exploration_ns::exploration_planner::odomCallback(const nav_msgs::Odometry &pose){
  
  robotOdom_ = pose;
  grid_->updateRobotOdom(robotOdom_);
  odomPub_.publish(robotOdom_);
}
void exploration_ns::exploration_planner::mapCallback(const nav_msgs::OccupancyGrid &map){
  
  gridMap_ = map;
  grid_->updateMap(gridMap_);
  mapPub_.publish(gridMap_);
}
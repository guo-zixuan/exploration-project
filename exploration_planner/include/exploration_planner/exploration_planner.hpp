# pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_world/octomap_manager.h>

namespace  exploration_ns{
  class exploration_planner{
    
    public:

    exploration_planner(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
    ~exploration_planner();
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber odomSub_;
    ros::Subscriber mapSub_;
    ros::Subscriber octomapSub_;

    ros::Publisher odomPub_;
    ros::Publisher mapPub_;
    ros::Publisher octomapPub_;

    volumetric_mapping::OctomapManager *manager_;
    nav_msgs::OccupancyGrid gridMap_;
    nav_msgs::Odometry robotOdom_;

    bool init();
    bool setParams();
    void odomCallback(const nav_msgs::Odometry &pose);
    void mapCallback(const nav_msgs::OccupancyGrid &map);

    private:

    std::string odomSubTopic_;
    std::string mapSubTopic_;

  };
}

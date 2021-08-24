#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_world/octomap_manager.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


namespace  exploration_ns{

  struct coordinate{
    int x;
    int y;
  };

  class grid{
    public:
      grid();
      ~grid();

      void init();
      void getMapParams();

      void getPotentialFrontier();

      void updateRobotOdom(const nav_msgs::Odometry& odom);
      void updateMap(const nav_msgs::OccupancyGrid& map);

      void convertOdom2Grid(const nav_msgs::Odometry& pose, coordinate& coor);
      void convertGrid2Index(const coordinate& coor, int& index);

      bool isIndexCorrect(const coordinate& coor);

    private:

    nav_msgs::OccupancyGrid map_;
    nav_msgs::Odometry robotPosition_;

    Eigen::Vector3d mapOriginT_;
    Eigen::Quaterniond mapOriginR_;
    double mapRes_;
    int mapWidth_;
    int mapHeight_;

  };
}
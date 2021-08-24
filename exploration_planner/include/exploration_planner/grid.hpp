#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_world/octomap_manager.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <pcl/point_cloud.h>
#include <ros/timer.h>

namespace  exploration_ns{

  typedef uint32_t index_t;
  typedef int16_t coord_t;

struct Cell
{
  Cell(const coord_t x=0, const coord_t y=0) : x(x), y(y) {}
  coord_t x;
  coord_t y;

};


  const int8_t UNOCCUPIED=0;
  const int8_t OCCUPIED=100;
  const int8_t UNKNOWN=255;

  class grid{
    public:
      grid(ros::NodeHandle& nh,ros::NodeHandle& nh_p);
      ~grid();

      void init();
      void getMapParams();

      void execute(const ros::TimerEvent &e);

      void getPotentialFrontier();

      void updateRobotOdom(const nav_msgs::Odometry& odom);
      void updateMap(const nav_msgs::OccupancyGrid& map);

      Cell convertOdom2Grid(const nav_msgs::Odometry& odom);

      inline
      index_t cellIndex (const nav_msgs::MapMetaData& info, const Cell& c)
      {
        if (!withinBounds(info, c)){
          return -1;
        }
          
        return c.x + c.y*info.width;
      }

      inline
      Cell indexCell (const nav_msgs::MapMetaData& info, const index_t ind)
      {
        const div_t result = div((int) ind, (int) info.width);
        return Cell(result.rem, result.quot);
      }


      inline
      tf::Transform mapToWorld (const nav_msgs::MapMetaData& info)
      {
        tf::Transform world_to_map;
        tf::poseMsgToTF (info.origin, world_to_map);
        return world_to_map;
      }

      inline
      tf::Transform worldToMap (const nav_msgs::MapMetaData& info)
      {
        return mapToWorld(info).inverse();
      }

      inline
      Cell pointCell (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
      {
        tf::Point pt;
        tf::pointMsgToTF(p, pt);
        tf::Point p2 = worldToMap(info)*pt;
        return Cell(std::floor(p2.x()/info.resolution), std::floor(p2.y()/info.resolution));
      }

      inline 
      index_t pointIndex (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
      {
        return cellIndex(info, pointCell(info, p));
      }

      inline
      geometry_msgs::Point cellCenter (const nav_msgs::MapMetaData& info, const Cell& c)
      {
        tf::Point pt((c.x+0.5)*info.resolution, (c.y+0.5)*info.resolution, 0.0);
        geometry_msgs::Point p;
        tf::pointTFToMsg(mapToWorld(info)*pt, p);
        return p;
      }

      inline
      bool withinBounds (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
      {
        return withinBounds(info, pointCell(info, p));
      }

      inline
      bool withinBounds (const nav_msgs::MapMetaData& info, const Cell& c)
      {
        return (c.x >= 0) && (c.y >= 0) && (c.x < (coord_t) info.width) && (c.y < (coord_t) info.height);
      }



    private:

    ros::Publisher potentialFrontierPub_;
    ros::Publisher unknownPointPub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer executeTimer_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::Odometry robotPosition_;
    Cell currentCoordinate_;

    Eigen::Vector3d mapOriginT_;
    Eigen::Quaterniond mapOriginR_;

    double mapRes_;
    int mapWidth_;
    int mapHeight_;

    bool getMapState_;
    double updateFrequency_;

  };
}
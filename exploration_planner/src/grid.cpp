#include<exploration_planner/grid.hpp>

using namespace exploration_ns;

exploration_ns::grid::grid(ros::NodeHandle& nh,ros::NodeHandle& nh_p):
nh_(nh),
nh_private_(nh_p),
currentCoordinate_(Cell(0,0)),
getMapState_(0),
updateFrequency_(10.0)
{
  init();
}

exploration_ns::grid::~grid(){

}

//初始化定时器；定期发布潜在的前沿点
void exploration_ns::grid::init(){
  
  unknownPointPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/unknown_points",100);

  if (updateFrequency_ > 0.0) {
    executeTimer_ =
        nh_private_.createTimer(ros::Duration(1.0 / updateFrequency_),
                                &exploration_ns::grid::execute, this);
  }

}

void exploration_ns::grid::execute(const ros::TimerEvent &e){
  if(getMapState_){
    
    getAllUnknownPoints();

  }else{
    ROS_INFO("waiting for 2d grid map");
  }
}

void exploration_ns::grid::getAllUnknownPoints(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr unknownPoints = 
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  for(int i=0;i<=map_.data.size();i++){
    
    if(map_.data[i] == UNKNOWN){

      geometry_msgs::Point p =  cellCenter(map_.info,indexCell(map_.info,i));

      if(isPotentialFrontier(p)){

        pcl::PointXYZ p_pcl(p.x,p.y,p.z);

        unknownPoints->push_back(p_pcl);

      }
    }
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*unknownPoints,msg);
  msg.header.frame_id = "/map";
  msg.header.stamp = ros::Time::now();

  unknownPointPub_.publish(msg);

}

bool exploration_ns::grid::isPotentialFrontier(const geometry_msgs::Point& p){
  
  Cell c = pointCell(map_.info,p);
  return isPotentialFrontier(c);
}

bool exploration_ns::grid::isPotentialFrontier(const Cell& c){
  
  for(int x=-1;x <= 1;x++ ){
    for(int y=-1;y <= 1;y++ ){
      Cell temp(c);
      temp.x += x;
      temp.y += y;

      index_t index = cellIndex(map_.info,temp);

      if(index == -1){
        continue;
      }

      if(map_.data[index] == exploration_ns::UNOCCUPIED){
        return true;
      }
    }
  }
  return false;
}

void exploration_ns::grid::updateMap(const nav_msgs::OccupancyGrid& map){
  
  this->map_ = map;
  getMapState_ = true;

}

void exploration_ns::grid::updateRobotOdom(const nav_msgs::Odometry& odom){
  
  robotPosition_ = odom;
  //update current coordinate
  currentCoordinate_ = convertOdom2Grid(robotPosition_);

}

Cell exploration_ns::grid::convertOdom2Grid(const nav_msgs::Odometry& odom){
  
  geometry_msgs::Point point;
  point.x = odom.pose.pose.position.x;
  point.y = odom.pose.pose.position.y;
  point.z = 0;

  //update current coordinate
  Cell coor = pointCell(map_.info,point);
  return coor;
}
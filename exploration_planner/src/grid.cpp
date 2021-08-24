#include<exploration_planner/grid.hpp>

using namespace exploration_ns;

exploration_ns::grid::grid(ros::NodeHandle& nh,ros::NodeHandle& nh_p):
nh_(nh),
nh_private_(nh_p),
currentCoordinate_(Cell(0,0)),
mapOriginT_(Eigen::Vector3d::Zero()),
mapOriginR_(Eigen::Quaterniond::Identity()),
mapRes_(0.0),
mapWidth_(0),
mapHeight_(0),
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
    
    getPotentialFrontier();

  }else{
    ROS_INFO("waiting for 2d grid map");
  }
}

void exploration_ns::grid::getPotentialFrontier(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr potentialFrontiers = 
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  for(int i=0;i<=map_.data.size();i++){
    
    if(map_.data[i] == -1){

      geometry_msgs::Point p =  cellCenter(map_.info,indexCell(map_.info,i));
      pcl::PointXYZ p_pcl(p.x,p.y,p.z);

      potentialFrontiers->push_back(p_pcl);
    }
  }



  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*potentialFrontiers,msg);
  msg.header.frame_id = "/map";
  msg.header.stamp = ros::Time::now();

  unknownPointPub_.publish(msg);

}

void exploration_ns::grid::updateMap(const nav_msgs::OccupancyGrid& map){
  
  this->map_ = map;

  this->mapHeight_ = map.info.height;
  this->mapWidth_ = map.info.width;
  this->mapRes_ = map.info.resolution;

  double t_x,t_y,t_z;
  t_x = map.info.origin.position.x;
  t_y = map.info.origin.position.y;
  t_z = map.info.origin.position.z;
  
  this->mapOriginT_ = Eigen::Vector3d(t_x,t_y,t_z);

  double q_x,q_y,q_z,q_w;
  q_x = map.info.origin.orientation.x;
  q_y = map.info.origin.orientation.y;
  q_z = map.info.origin.orientation.z;
  q_w = map.info.origin.orientation.w;

  this->mapOriginR_ = Eigen::Quaterniond(q_w,q_x,q_y,q_z);

  getMapState_ = true;
}

void exploration_ns::grid::updateRobotOdom(const nav_msgs::Odometry& odom){
  
  robotPosition_ = odom;
  currentCoordinate_ = convertOdom2Grid(robotPosition_);

  //update current coordinate
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
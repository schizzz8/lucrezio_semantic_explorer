#include "lucrezio_explorer.h"

LucrezioExplorer::LucrezioExplorer(ros::NodeHandle nh_):
  _nh(nh_){

  _robot_pose_sub = _nh.subscribe("/gazebo/model_states",
                                   1000,
                                   &LucrezioExplorer::robotPoseCallback,
                                   this);
  _map_sub = _nh.subscribe("/map",
                           1000,
                           &LucrezioExplorer::mapCallback,
                           this);

  _got_map = false;
  _got_pose= false;
}

bool LucrezioExplorer::detectFrontiers(){

  if(!_got_map || !_got_pose)
    return false;

  _got_map=false;
  _got_pose=false;

  std::cerr << std::endl;
  std::cerr << "Grid" << std::endl;
  std::cerr << "Resolution: " << _resolution << std::endl;
  std::cerr << "Dimesions: " << _occupancy_grid.rows << "x" << _occupancy_grid.cols << std::endl;
  std::cerr << "Origin: " << _origin.transpose() << std::endl;
  std::cerr << std::endl;
  std::cerr << "Robot" << std::endl;
  std::cerr << "Position: " << _robot_pose.translation().head(2).transpose() << std::endl;
  std::cerr << "Orientation: " << _robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;

  //sanity check
  init();

  //compute
  std::cerr << "a";
  computeFrontierPoints();
  std::cerr << "b";
  computeFrontierRegions();
  std::cerr << "c";
  computeFrontierCentroids();
  std::cerr << "d";
  rankFrontierCentroids();
  std::cerr << "e";

  if(_frontier_scored_centroids.empty())
    return false;

  //visualize output
  drawFrontiers();

  return true;
}

bool LucrezioExplorer::listenRobotPose(Eigen::Isometry3f &robot_pose){

  tf::TransformListener listener;
  tf::StampedTransform robot_tf;
  try {
    listener.waitForTransform("map",
                              "base_footprint",
                              ros::Time(0),
                              ros::Duration(3));
    listener.lookupTransform("map",
                             "base_footprint",
                             ros::Time(0),
                             robot_tf);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  robot_pose.setIdentity();
  robot_pose.translation().x()=robot_tf.getOrigin().x();
  robot_pose.translation().y()=robot_tf.getOrigin().y();
  robot_pose.translation().z()=robot_tf.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = robot_tf.getRotation();
  q.x()= tq.x();
  q.y()= tq.y();
  q.z()= tq.z();
  q.w()= tq.w();
  robot_pose.linear()=q.toRotationMatrix();

  return true;
}

bool LucrezioExplorer::receiveOccupancyGridMsg(const std::string &map_topic,
                                               float duration,
                                               float &resolution,
                                               Eigen::Vector2f &origin,
                                               srrg_core::UnsignedCharImage &occupancy_grid){

  boost::shared_ptr<nav_msgs::OccupancyGrid const> occupancy_grid_msg_ptr;
  occupancy_grid_msg_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid> (map_topic, ros::Duration (duration));

  if(occupancy_grid_msg_ptr == NULL){
    ROS_ERROR("No occupancy_grid message received!");
    return false;
  } else {

    resolution = occupancy_grid_msg_ptr->info.resolution;
    origin << occupancy_grid_msg_ptr->info.origin.position.x,occupancy_grid_msg_ptr->info.origin.position.y;

    //convert to cv::Mat
    int width = occupancy_grid_msg_ptr->info.width;
    int height = occupancy_grid_msg_ptr->info.height;
    occupancy_grid.create(height,width);
    for (int i = 0, i_rev = height - 1; i < height; i++, i_rev--)
      for (int j = 0; j < width; j++)
        switch (occupancy_grid_msg_ptr->data[i_rev*width + j]) {
          default:
          case -1:
            occupancy_grid.data[i*width + j] = Occupancy::UNKNOWN;
            break;
          case 0:
            occupancy_grid.data[i*width + j] = Occupancy::FREE;
            break;
          case 100:
            occupancy_grid.data[i*width + j] = Occupancy::OCCUPIED;
            break;
        }

    return true;
  }
}

void LucrezioExplorer::robotPoseCallback(const gazebo_msgs::ModelStates::ConstPtr &robot_pose_msg){
  _last_pose_timestamp = ros::Time::now();

  const std::vector<std::string> &names = robot_pose_msg->name;
  for(size_t i=0; i<names.size(); ++i){
    if(names[i].compare("robot") == 0){
      const geometry_msgs::Pose robot_pose = robot_pose_msg->pose[i];
      _robot_pose=poseMsg2eigen(robot_pose);
      break;
    }
  }
  _got_pose = true;
}

void LucrezioExplorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg){

  _last_map_timestamp = map_msg->header.stamp;

  _resolution = map_msg->info.resolution;
  _origin = Eigen::Vector2f(map_msg->info.origin.position.x,map_msg->info.origin.position.y);

  //convert to cv::Mat
  int width = map_msg->info.width;
  int height = map_msg->info.height;
  _occupancy_grid.create(height,width);
  for (int i = 0, i_rev = height - 1; i < height; i++, i_rev--)
    for (int j = 0; j < width; j++)
      switch (map_msg->data[i_rev*width + j]) {
        default:
        case -1:
          _occupancy_grid.data[i*width + j] = Occupancy::UNKNOWN;
          break;
        case 0:
          _occupancy_grid.data[i*width + j] = Occupancy::FREE;
          break;
        case 100:
          _occupancy_grid.data[i*width + j] = Occupancy::OCCUPIED;
          break;
      }
  _got_map = true;
}

Eigen::Isometry3f LucrezioExplorer::poseMsg2eigen(const geometry_msgs::Pose &p){
  Eigen::Isometry3f iso;
  iso.translation().x()=p.position.x;
  iso.translation().y()=p.position.y;
  iso.translation().z()=p.position.z;
  Eigen::Quaternionf q;
  q.x()=p.orientation.x;
  q.y()=p.orientation.y;
  q.z()=p.orientation.z;
  q.w()=p.orientation.w;
  iso.linear()=q.toRotationMatrix();
  return iso;
}

//bool LucrezioExplorer::detectFrontiers(){

//  //setup frontier detector
//  setup();

//  //listen to robot pose
//  Eigen::Isometry3f robot_pose;
//  if(listenRobotPose(robot_pose))
//    setRobotPose(robot_pose);
//  else {
//    ROS_ERROR("Couldn't receive robot pose!");
//    return false;
//  }

//  //receive current map
//  float resolution;
//  Eigen::Vector2f origin;
//  srrg_core::UnsignedCharImage occupancy_grid;
//  if(receiveOccupancyGridMsg("/map",
//                             3,
//                             resolution,
//                             origin,
//                             occupancy_grid)){
//    setResolution(resolution);
//    setOrigin(origin);
//    setMap(occupancy_grid);
//  } else {
//    ROS_ERROR("Couldn't receive occupancy grid!");
//    return false;
//  }

//  std::cerr << std::endl;
//  std::cerr << "Grid" << std::endl;
//  std::cerr << "Resolution: " << resolution << std::endl;
//  std::cerr << "Dimesions: " << occupancy_grid.rows << "x" << occupancy_grid.cols << std::endl;
//  std::cerr << "Origin: " << origin.transpose() << std::endl;
//  std::cerr << std::endl;
//  std::cerr << "Robot" << std::endl;
//  std::cerr << "Position: " << robot_pose.translation().head(2).transpose() << std::endl;
//  std::cerr << "Orientation: " << robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;

//  //sanity check
//  init();

//  //compute
//  computeFrontierPoints();
//  computeFrontierRegions();
//  computeFrontierCentroids();
//  rankFrontierCentroids();

//  if(_frontier_scored_centroids.empty())
//    return false;

//  //visualize output
//  drawFrontiers();

//  return true;
//}

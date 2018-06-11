#include "lucrezio_explorer.h"

using namespace move_base_msgs;

void LucrezioExplorer::computeNextPoses(){

  //listen to robot pose
  Eigen::Isometry3f robot_pose;
  if(listenRobotPose(robot_pose))
    setRobotPose(robot_pose);

  //receive current map
  float resolution;
  Eigen::Vector2f origin;
  srrg_core::UnsignedCharImage occupancy_grid;
  if(receiveOccupancyGridMsg("/map",
                             1,
                             resolution,
                             origin,
                             occupancy_grid)){
    setResolution(resolution);
    setOrigin(origin);
    setMap(occupancy_grid);
  }

  std::cerr << std::endl;
  std::cerr << "Grid" << std::endl;
  std::cerr << "Resolution: " << resolution << std::endl;
  std::cerr << "Dimesions: " << occupancy_grid.rows << "x" << occupancy_grid.cols << std::endl;
  std::cerr << "Origin: " << origin.transpose() << std::endl;
  std::cerr << std::endl;
  std::cerr << "Robot" << std::endl;
  std::cerr << "Position: " << robot_pose.translation().head(2).transpose() << std::endl;
  std::cerr << "Orientation: " << robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;

  //compute goal
  init();

  computeFrontierPoints();
  computeFrontierRegions();
  computeFrontierCentroids();
  rankFrontierCentroids();
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

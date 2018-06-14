#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <visualization_msgs/Marker.h>
#include <frontier_detector/frontier_detector.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector2f &next_pose);
visualization_msgs::Marker makeRVizMarker(const Eigen::Vector2f &next_pose);

bool listenRobotPose(Eigen::Isometry3f &robot_pose);
bool receiveOccupancyGridMsg(float &resolution,
                             Eigen::Vector2f &origin,
                             srrg_core::UnsignedCharImage &occupancy_grid);
void showOutput(const Eigen::Vector3f &position,
                const float resolution,
                const Eigen::Vector2f & origin,
                const srrg_core::UnsignedCharImage &gray_image,
                ScoredCellQueue scored_centroids);


int main(int argc, char **argv){

  ros::init(argc,argv,"lucrezio_explorer");
  ros::NodeHandle nh;

  MoveBaseClient ac("move_base",true);

  ros::Publisher marker_pub;
  marker_pub = nh.advertise<visualization_msgs::Marker>("goal_visualization_marker",1);

  Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
  float resolution = 0.0f;
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();
  srrg_core::UnsignedCharImage occupancy_grid;

  FrontierDetector detector;

  bool success = false;

  while(ros::ok() && !success){

    if(listenRobotPose(robot_pose)){
      ROS_INFO("Received robot pose!");
      std::cerr << "Position: " << robot_pose.translation().head(2).transpose() << std::endl;
      std::cerr << "Orientation: " << robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;
    }

    if(receiveOccupancyGridMsg(resolution,origin,occupancy_grid)){
      ROS_INFO("Received occupancy grid!");
      std::cerr << "Resolution: " << resolution << std::endl;
      std::cerr << "Dimesions: " << occupancy_grid.rows << "x" << occupancy_grid.cols << std::endl;
      std::cerr << "Origin: " << origin.transpose() << std::endl;
    }

    ROS_INFO("Calling detector!");
    detector.setup();
    detector.setRobotPose(robot_pose);
    detector.setResolution(resolution);
    detector.setOrigin(origin);
    detector.setMap(occupancy_grid);
    detector.init();

    detector.compute();
    ScoredCellQueue scored_centroids = detector.frontierScoredCentroids();

    ROS_INFO("Visualizing output!");
    showOutput(robot_pose.translation(),resolution,origin,occupancy_grid,scored_centroids);

    if(scored_centroids.empty()){
      success = true;
      continue;
    }

    bool reached = false;

    while(!reached && !scored_centroids.empty()){

      ROS_INFO("Waiting for move_base action server to start.");
      ac.waitForServer();

      const ScoredCell &current = scored_centroids.top();
      scored_centroids.pop();

      float next_pose_x = current.cell.x()*resolution + origin.x();
      float next_pose_y = (occupancy_grid.rows - current.cell.y())*resolution + origin.y();
      Eigen::Vector2f next_pose (next_pose_x,next_pose_y);

      ROS_INFO("Action server started, sending goal.");
      move_base_msgs::MoveBaseGoal goal = makeMoveBaseGoal(next_pose);
      ac.sendGoal(goal);

      //visualize next pose (RViz)
      visualization_msgs::Marker marker = makeRVizMarker(next_pose);
      marker_pub.publish(marker);

      //wait for the action server to return
      bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
      if (finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state.toString().compare("SUCCEEDED") == 0)
          reached = true;
      }
      else{
        ROS_INFO("Action did not finish before the time out.");
      }
    }

//    if(scored_centroids.empty()){
//      success = true;
//      continue;
//    }
  }

  ROS_INFO("Exploration complete!");

  return 0;
}

bool listenRobotPose(Eigen::Isometry3f &robot_pose){
  tf::TransformListener listener;
  tf::StampedTransform robot_tf;
  try {
    listener.waitForTransform("map",
                              "base_link",
                              ros::Time(0),
                              ros::Duration(3));
    listener.lookupTransform("map",
                             "base_link",
                             ros::Time(0),
                             robot_tf);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
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

bool receiveOccupancyGridMsg(float &resolution,
                             Eigen::Vector2f &origin,
                             srrg_core::UnsignedCharImage &occupancy_grid){

  boost::shared_ptr<nav_msgs::OccupancyGrid const> occupancy_grid_msg_ptr;
  occupancy_grid_msg_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid> ("/map", ros::Duration (10));

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

void showOutput(const Eigen::Vector3f &position,
                const float resolution,
                const Eigen::Vector2f &origin,
                const srrg_core::UnsignedCharImage &gray_image,
                ScoredCellQueue scored_centroids){

  srrg_core::RGBImage rgb_image;
  cv::cvtColor(gray_image,rgb_image,CV_GRAY2BGR);

  Eigen::Vector2i pos_cell = ((position.head(2)-origin)/resolution).cast<int>();
  cv::circle(rgb_image,cv::Point2i(pos_cell.x(),rgb_image.rows-pos_cell.y()),3,cv::Scalar(0,0,255),3);

  int count = 0;
  while (!scored_centroids.empty()) {
    ScoredCell centroid = scored_centroids.top();
    cv::Point2i cell(centroid.cell.x(),centroid.cell.y());
    cv::circle(rgb_image,cell,4,cv::Scalar(255,0,0),2);
    std::ostringstream ss;
    ss << count;
    cv::putText(rgb_image, ss.str(), cell, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
    count++;
    scored_centroids.pop();
  }

  cv::imshow("output",rgb_image);
  cv::waitKey(1);
}

move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector2f & next_pose){

  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.frame_id = "/map";
  goal_msg.target_pose.header.stamp = ros::Time::now();

  goal_msg.target_pose.pose.position.x = next_pose.x();
  goal_msg.target_pose.pose.position.y = next_pose.y();
  goal_msg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  return goal_msg;
}

visualization_msgs::Marker makeRVizMarker(const Eigen::Vector2f & next_pose){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = next_pose.x();
  marker.pose.position.y = next_pose.y();
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return marker;
}

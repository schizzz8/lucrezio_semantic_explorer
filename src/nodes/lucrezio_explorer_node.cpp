#include <iostream>

#include <lucrezio_explorer/lucrezio_explorer.h>

#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <visualization_msgs/Marker.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef std::vector<geometry_msgs::PoseStamped_<std::allocator<void> >,
std::allocator<geometry_msgs::PoseStamped_<std::allocator<void> > > > PoseStampedVector;
geometry_msgs::PoseStamped getPoseFromTransform(const tf::StampedTransform &tf);
geometry_msgs::PoseStamped getPoseFromPosition(const Eigen::Vector2f &tf);
move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector2f &next_pose);
visualization_msgs::Marker makeRVizMarker(const Eigen::Vector2f &next_pose);
//void showNextPose(const ScoredCell &next_pose);

int main(int argc, char **argv){

  ros::init(argc, argv, "lucrezio_explorer_node");

  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",1);

  LucrezioExplorer explorer;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  MoveBaseClient ac("move_base",true);

  //start execution loop
  ros::Rate loop_rate(1);
  while (ros::ok()){

    //listen to robot pose
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
    }
    geometry_msgs::PoseStamped robot_pose = getPoseFromTransform(robot_tf);

    //compute next pose
    explorer.computeNextPoses();

    const ScoredCellQueue &next_poses = explorer.frontierScoredCentroids();
    Eigen::Vector2f next_pose;
    while (!next_poses.empty()) {
      const ScoredCell &current = next_poses.top();

      float next_pose_x = current.cell.x()*explorer.resolution() + explorer.origin().x();
      float next_pose_y = (explorer.occupancyGrid().rows - current.cell.y())*explorer.resolution() + explorer.origin().y();

      next_pose << next_pose_x,next_pose_y;
      geometry_msgs::PoseStamped goal_pose = getPoseFromPosition(next_pose);

      nav_msgs::GetPlan srv;
      srv.request.start = robot_pose;
      srv.request.goal = goal_pose;
      srv.request.tolerance = 0.5;

      if (client.call(srv)){
        nav_msgs::Path path = srv.response.plan;
        PoseStampedVector poses = path.poses;

        //check if exists a path to the current pose
        if(!poses.empty())
          break;
      } else {
        ROS_ERROR("Failed to call service make_plan");
      }
    }

    //visualize next pose (RViz)
    visualization_msgs::Marker marker = makeRVizMarker(next_pose);
    marker_pub.publish(marker);

    ROS_INFO("Waiting for move_base action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal goal = makeMoveBaseGoal(next_pose);
    ac.sendGoal(goal);

    //wait for the action server to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
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

//void showNextPose(const ScoredCell &next_pose){

//  //map
//  srrg_core::RGBImage occupancy_rgb;
//  cv::cvtColor(_detector.occupancyGrid(),occupancy_rgb,CV_GRAY2BGR);

//  //robot position
//  const float &resolution = _detector.resolution();
//  const Eigen::Vector2f &grid_origin = _detector.origin();
//  Eigen::Vector2i robot_position = ((_detector.robotPose().translation().head(2)-grid_origin)/resolution).cast<int>();
//  cv::Point2i robot(robot_position.x(),occupancy_rgb.rows - robot_position.y());
//  cv::circle(occupancy_rgb,robot,4,cv::Scalar(0,0,255),2);

//  //next pose
//  cv::Point2i next(next_pose.cell.x(),next_pose.cell.y());
//  cv::circle(occupancy_rgb,next,4,cv::Scalar(255,0,0),2);
//  float score = next_pose.score;
//  std::ostringstream ss;
//  ss << score;
//  cv::putText(occupancy_rgb, ss.str(), next, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));

//  cv::imshow("explorer",occupancy_rgb);
//  cv::waitKey(10);
//}

geometry_msgs::PoseStamped getPoseFromTransform(const tf::StampedTransform &tf) {

  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion quat;
  tf::Quaternion tf_quat;
  tf_quat = tf.getRotation();
  quat.x = tf_quat.x();
  quat.y = tf_quat.y();
  quat.z = tf_quat.z();
  quat.w = tf_quat.w();
  pose.pose.orientation = quat;

  tf::Vector3 tf_vec;
  geometry_msgs::Point pt;
  tf_vec = tf.getOrigin();
  pt.x = tf_vec.getX();
  pt.y = tf_vec.getY();
  pt.z = tf_vec.getZ();
  pose.pose.position= pt;
  pose.header.frame_id = tf.frame_id_;
  pose.header.stamp = tf.stamp_;
  return pose;
}

geometry_msgs::PoseStamped getPoseFromPosition(const Eigen::Vector2f &tf){

  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion quat;
  quat.x = 0;
  quat.y = 0;
  quat.z = 0;
  quat.w = 1;
  pose.pose.orientation = quat;

  geometry_msgs::Point pt;
  pt.x = tf.x();
  pt.y = tf.y();
  pt.z = 0;
  pose.pose.position= pt;
  pose.header.frame_id = "/map";
  pose.header.stamp = ros::Time::now();
  return pose;
}

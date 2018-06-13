#include <iostream>

#include <lucrezio_explorer/lucrezio_explorer.h>

#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <visualization_msgs/Marker.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector2f &next_pose);
visualization_msgs::Marker makeRVizMarker(const Eigen::Vector2f &next_pose);

int main(int argc, char **argv){

  ros::init(argc, argv, "lucrezio_explorer_node");

  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  marker_pub = nh.advertise<visualization_msgs::Marker>("goal_visualization_marker",1);

  LucrezioExplorer explorer(nh);

  MoveBaseClient ac("move_base",true);

  //start execution loop
  ros::Rate loop_rate(1);
  while (ros::ok()){

    //    ROS_INFO("\nDetect frontiers!");
    if(explorer.detectFrontiers()){

      ScoredCellQueue frontiers = explorer.frontierScoredCentroids();
      Eigen::Vector2f next_pose;
      ROS_INFO("Detected %lu frontiers!",frontiers.size());

      bool exit = false;

      while (!frontiers.empty() && !exit) {

        const ScoredCell &current = frontiers.top();

        float next_pose_x = current.cell.x()*explorer.resolution() + explorer.origin().x();
        float next_pose_y = (explorer.occupancyGrid().rows - current.cell.y())*explorer.resolution() + explorer.origin().y();
        next_pose << next_pose_x,next_pose_y;

        ROS_INFO("Waiting for move_base action server to start.");
        ac.waitForServer();

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
          if(state.toString().compare("ABORTED") == 0)
            frontiers.pop();
          else
            exit = true;
        }
        else{
          ROS_INFO("Action did not finish before the time out.");
          frontiers.pop();
        }
      }
    }

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

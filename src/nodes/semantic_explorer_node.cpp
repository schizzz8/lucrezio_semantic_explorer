#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lucrezio_semantic_mapper/SemanticMap.h>
#include <Eigen/Geometry>
#include <semantic_explorer/semantic_explorer.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool listenRobotPose(Eigen::Isometry3f &robot_pose);
bool receiveSemanticMap(size_t &num_models,const SemanticMap* semantic_map);

move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector3f &next_pose);
visualization_msgs::Marker makeRVizMarker(const Eigen::Vector3f &next_pose);

int main(int argc, char **argv){

  ros::init(argc,argv,"semantic_explorer");
  ros::NodeHandle nh;

  MoveBaseClient ac("move_base",true);

  ros::Publisher marker_pub;
  marker_pub = nh.advertise<visualization_msgs::Marker>("goal_visualization_marker",1);

  Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
  size_t num_models;
  const SemanticMap* semantic_map(new SemanticMap);

  SemanticExplorer explorer;

  bool exit = false;

  while(ros::ok() && !exit){
    if(!listenRobotPose(robot_pose))
      continue;
    else{
      ROS_INFO("Received robot pose!");
      std::cerr << "Position: " << robot_pose.translation().head(2).transpose() << std::endl;
      std::cerr << "Orientation: " << robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;
    }

    if(!receiveSemanticMap(num_models,semantic_map))
      continue;
    else{
      ROS_INFO("Received semantic map!");
      std::cerr << "Num models: " << num_models << std::endl;
    }

    ROS_INFO("Calling semantic explorer!");
    explorer.setRobotPose(robot_pose);
    explorer.setObjects(semantic_map);

    if(explorer.findNearestObject()){

      ROS_INFO("Waiting for move_base action server to start.");
      ac.waitForServer();

      Vector3fVector poses = explorer.computePoses();

      for(int i=0; i<4; ++i){

        std::cerr << "Next pose: " << poses[i].transpose() << std::endl;

        move_base_msgs::MoveBaseGoal goal = makeMoveBaseGoal(poses[i]);
        ROS_INFO("Action server started, sending goal.");
        ac.sendGoal(goal);

        //visualize next pose (RViz)
        visualization_msgs::Marker marker = makeRVizMarker(poses[i]);
        marker_pub.publish(marker);

        //wait for the action server to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout){
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
          if(state.toString().compare("SUCCEEDED") == 0)
            ros::Duration(4).sleep();
        }
      }

      explorer.setProcessed();
    } else
      exit=true;
  }

  delete semantic_map;

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

bool receiveSemanticMap(size_t & num_models, const SemanticMap * semantic_map){
  boost::shared_ptr<lucrezio_semantic_mapper::SemanticMap const> semantic_map_msg_ptr;
  semantic_map_msg_ptr = ros::topic::waitForMessage<lucrezio_semantic_mapper::SemanticMap> ("/semantic_map", ros::Duration (10));
  if(!semantic_map_msg_ptr){
    ROS_ERROR("No semantic_map message received!");
    return false;
  }

  num_models = semantic_map_msg_ptr->objects.size();
  if(!num_models){
    ROS_ERROR("Received empty semantic map!");
    return false;
  }

  for(size_t i=0; i<num_models; ++i){
    const lucrezio_semantic_mapper::Object &o = semantic_map_msg_ptr->objects[i];
    ObjectPtr obj_ptr(new Object(o.type,
                                 Eigen::Vector3f(o.position.x,o.position.y,o.position.z)));
    semantic_map->addObject(obj_ptr);
  }
  return true;
}

move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector3f & next_pose){

  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.frame_id = "/map";
  goal_msg.target_pose.header.stamp = ros::Time::now();

  goal_msg.target_pose.pose.position.x = next_pose.x();
  goal_msg.target_pose.pose.position.y = next_pose.y();
  goal_msg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_pose.z());

  return goal_msg;
}

visualization_msgs::Marker makeRVizMarker(const Eigen::Vector3f & next_pose){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = next_pose.x();
  marker.pose.position.y = next_pose.y();
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(next_pose.z());

  marker.scale.x = 0.25;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return marker;
}

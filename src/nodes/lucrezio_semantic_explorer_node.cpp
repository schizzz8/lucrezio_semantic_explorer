#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lucrezio_semantic_mapper/SemanticMap.h>
#include <Eigen/Geometry>
#include <semantic_explorer/semantic_explorer.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool listenRobotPose(Eigen::Isometry3f &robot_pose);
bool receiveSemanticMap(size_t &num_models,SemanticMap &semantic_map);

int main(int argc, char **argv){

  ros::init(argc,argv,"semantic_explorer");
  ros::NodeHandle nh;

  MoveBaseClient ac("move_base",true);

  Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
  size_t num_models;
  SemanticMap semantic_map;

  SemanticExplorer explorer;

  bool success = false;

  while(ros::ok() && !success){
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
    explorer.setSemanticMap(semantic_map);
    explorer.init();

//    int nearest_idx = explorer.findNearestObject();

//    if(nearest_idx<0){
//      success=true;
//      continue;
//    }

  }

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

bool receiveSemanticMap(size_t & num_models, SemanticMap &semantic_map){
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
    semantic_map.addObject(obj_ptr);
  }
  return true;
}

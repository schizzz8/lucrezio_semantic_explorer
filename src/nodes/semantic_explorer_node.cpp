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
bool receiveSemanticMap(int &num_models,ObjectPtrVector& semantic_map);

move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector3f &next_pose);
visualization_msgs::Marker makeRVizMarker(const Eigen::Vector3f &next_pose);

int main(int argc, char **argv){

  ros::init(argc,argv,"semantic_explorer");
  ros::NodeHandle nh;

  MoveBaseClient ac("move_base",true);

  ros::Publisher marker_pub;
  marker_pub = nh.advertise<visualization_msgs::Marker>("goal_visualization_marker",1);

  Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.translation() = Eigen::Vector3f(0.0,0.0,0.5);
  int num_models=0;
  ObjectPtrVector semantic_map;

  SemanticExplorer explorer;
  int unn_threshold;

  while(!listenRobotPose(robot_pose) || !receiveSemanticMap(num_models,semantic_map)){
    ROS_INFO("Waiting to for robot_pose and semantic_map to start exploration...");
  }
  explorer.setCameraPose(robot_pose*camera_offset);
  explorer.setObjects(semantic_map);
  ROS_INFO("Starting exploration!!!");

  bool exit = false;
  while(ros::ok() && !exit) {

    //find nearest object
    ObjectPtr nearest_object=0;
    if(!explorer.findNearestObject(nearest_object)){
      ROS_INFO("No objects to process!");
      exit=true; //no more objects to process
      continue;
    }
    std::cerr << "Processing: " << nearest_object->model() << std::endl;

    bool reconstructed=false;
    while(!reconstructed){
      //listen robot pose
      if(!listenRobotPose(robot_pose))
        continue;
      else{
        ROS_INFO("Received robot pose!");
        std::cerr << "Position: " << robot_pose.translation().head(2).transpose() << std::endl;
        std::cerr << "Orientation: " << robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;
        explorer.setCameraPose(robot_pose*camera_offset);
      }

      //receive semantic map
      if(!receiveSemanticMap(num_models,semantic_map))
        continue;
      else{
        ROS_INFO("Received semantic map!");
        std::cerr << "Num models: " << num_models << std::endl;
        explorer.setObjects(semantic_map);
      }

      //generate candidate views
      Isometry3fVector candidate_views = explorer.generateCandidateViews(nearest_object);

      //compute NBV
      explorer.computeNBV_Jose(candidate_views,nearest_object);     //computeNBV_Jose
      ScoredPoseQueue tmp_q = explorer.views();
      bool reached=false;
      while(!tmp_q.empty() && !reached){

        //current NBV
        ScoredPose view = tmp_q.top();
        Eigen::Vector3f nbv = view.pose;
        int unn=view.score;
        std::cerr << "NBV: " << nbv.transpose() << std::endl;
        std::cerr << "Unn max: " << unn << std::endl;

        //check if object is reconstructed
        if(unn <= unn_threshold){
          reconstructed = true;
          ROS_INFO("%s: processed!",nearest_object->model().c_str());
          explorer.setProcessed(nearest_object);
          break;
        }

        //visualize next pose (RViz)
        visualization_msgs::Marker marker = makeRVizMarker(nbv);
        marker_pub.publish(marker);

        //send move_base goal
        ROS_INFO("Waiting for move_base action server to start.");
        ac.waitForServer();
        move_base_msgs::MoveBaseGoal goal = makeMoveBaseGoal(nbv);
        ROS_INFO("Action server started, sending goal.");
        ac.sendGoal(goal);

        //wait for the action server to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout){
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
          if(state.toString() == "SUCCEEDED")
            reached=true;
          if(state.toString() == "ABORTED")
            tmp_q.pop();
        }
        tmp_q.pop();
      }
      if(tmp_q.empty()){
        reconstructed = true;
        ROS_INFO("%s: processed!",nearest_object->model().c_str());
        explorer.setProcessed(nearest_object);
      }
    }
  }

  //free memory
  for (ObjectPtrVector::iterator it = semantic_map.begin(); it != semantic_map.end(); ++it)
    delete (*it);
  semantic_map.clear();

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

bool receiveSemanticMap(int& num_models, ObjectPtrVector& semantic_map){
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

  semantic_map.resize(num_models);
  for(int i=0; i<num_models; ++i){
    const lucrezio_semantic_mapper::Object &o = semantic_map_msg_ptr->objects[i];
    semantic_map[i] = new Object(o.type,
                                 Eigen::Vector3f(o.position.x,o.position.y,o.position.z),
                                 Eigen::Vector3f(o.min.x,o.min.y,o.min.z),
                                 Eigen::Vector3f(o.max.x,o.max.y,o.max.z),
                                 Eigen::Vector3f(o.color.x,o.color.y,o.color.z),
                                 o.cloud_filename,
                                 o.octree_filename,
                                 o.fre_voxel_cloud_filename,
                                 o.occ_voxel_cloud_filename);
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

  std::cerr << "Move Base Goal: " << goal_msg.target_pose.pose.position.x << " ";
  std::cerr << goal_msg.target_pose.pose.position.y << " ";
  std::cerr << goal_msg.target_pose.pose.orientation << std::endl;

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

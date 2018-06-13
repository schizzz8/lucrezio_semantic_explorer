#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
#include <actionlib/client/simple_action_client.h>

#include <frontier_detector/frontier_detector.h>

class LucrezioExplorer : public FrontierDetector {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LucrezioExplorer(ros::NodeHandle nh_);

    bool detectFrontiers();

    void robotPoseCallback(const gazebo_msgs::ModelStates::ConstPtr &robot_pose_msg);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);

  protected:
    ros::NodeHandle _nh;

    ros::Subscriber _robot_pose_sub;
//    Eigen::Isometry3f _robot_transform;
    ros::Time _last_pose_timestamp;

    ros::Subscriber _map_sub;
//    float _resolution;
//    Eigen::Vector2f _origin;
//    srrg_core::UnsignedCharImage _occupancy_grid;
    ros::Time _last_map_timestamp;

    bool _got_map;
    bool _got_pose;

  private:

    Eigen::Isometry3f poseMsg2eigen(const geometry_msgs::Pose& p);

    bool listenRobotPose(Eigen::Isometry3f &robot_pose);
    bool receiveOccupancyGridMsg(const std::string &map_topic,
                                 float duration,
                                 float &resolution,
                                 Eigen::Vector2f &origin,
                                 srrg_core::UnsignedCharImage &occupancy_grid);

};

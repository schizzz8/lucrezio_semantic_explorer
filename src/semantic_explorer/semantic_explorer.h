#pragma once

#include <semantic_mapper/semantic_map.h>

class SemanticExplorer{

  typedef std::vector<Eigen::Vector3f> Vector3fVector;
  typedef std::vector<bool> BoolVector;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  inline void setRobotPose(const Eigen::Isometry3f &robot_pose_){_robot_pose=robot_pose_;}

  void setSemanticMap(const SemanticMap &semantic_map_);

  void init();

  Object findNearestObject() const;

  void computePoses(int i);
  inline const Vector3fVector& poses() const {return _poses;}

protected:
  Eigen::Isometry3f _robot_pose;
  ObjectSet _semantic_map;
  Vector3fVector _poses;
};

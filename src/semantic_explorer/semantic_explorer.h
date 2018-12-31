#pragma once
#include <queue>
#include <semantic_mapper/object.h>

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > Isometry3fVector;
typedef std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>,
Eigen::aligned_allocator<std::pair<Eigen::Vector3f,Eigen::Vector3f> > > Vector3fPairVector;
typedef std::vector<std::string> StringVector;

struct ScoredPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool operator<(const ScoredPose& sc) const {
    return score < sc.score;
  }
  float score;
  Eigen::Vector3f pose;
};
typedef std::priority_queue<ScoredPose> ScoredPoseQueue;

class SemanticExplorer{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SemanticExplorer();

  inline void setCameraPose(const Eigen::Isometry3f &camera_pose_){_camera_pose=camera_pose_;}

  void setObjects(const ObjectPtrVector& semantic_map);

  bool findNearestObject(ObjectPtr& nearest_object);

  Isometry3fVector generateCandidateViews(const ObjectPtr& nearest_object);
  Isometry3fVector generateCandidateViews_Jose(const ObjectPtr& nearest_object);
  void computeNBV(const Isometry3fVector& candidate_views, const ObjectPtr& nearest_object);
  std::vector<int> computeNBV_Jose(const Isometry3fVector& candidate_views, const ObjectPtr& nearest_object, octomap::OcTree& unknown);
  void setProcessed(const ObjectPtr& nearest_object);

  inline const Vector3fPairVector& rays() const {return _rays;}

  inline const ScoredPoseQueue& views() const {return _views;}

protected:
  Eigen::Isometry3f _camera_pose;
  StringObjectPtrMap _objects;
  StringVector _processed;
  int _N;
  float _radius;
  Vector3fPairVector _rays;
  ScoredPoseQueue _views;

private:
  Eigen::Isometry3f v2t(const Eigen::Vector3f& v){
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation() = Eigen::Vector3f(v.x(),v.y(),0.6);
    T.linear() = Eigen::AngleAxisf(v.z(),Eigen::Vector3f::UnitZ()).matrix();
    return T;
  }

  Eigen::Vector3f t2v(const Eigen::Isometry3f& T){
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    v.x() = T.translation().x();
    v.y() = T.translation().y();
    Eigen::Vector3f euler = T.linear().eulerAngles(0,1,2);
    v.z() = euler.z();
    return v;
  }
};

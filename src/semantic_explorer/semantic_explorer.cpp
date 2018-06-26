#include "semantic_explorer.h"

void SemanticExplorer::init(){
  assert(_semantic_map.size() !=0 && "[SemanticExplorer][init] Empty semantic map!");
  _poses.clear();
}

void SemanticExplorer::setSemanticMap(const SemanticMap &semantic_map_){
  for(size_t i=0; i<semantic_map_.size(); ++i){
    const Object &o = *(semantic_map_[i]);
    ObjectSet::iterator it = _semantic_map.find(o);

    if(it!=_semantic_map.end())
      continue;

    _semantic_map.insert(o);
  }
}

Object SemanticExplorer::findNearestObject() const{
  Object nearest_object;
  float min_dist = std::numeric_limits<float>::max();

  for(ObjectSet::iterator it=_semantic_map.begin(); it!=_semantic_map.end(); ++it){
    const Object& o = *it;
    float dist=(o.position()-_robot_pose.translation()).norm();
    if(dist<min_dist){
      min_dist=dist;
      nearest_object=o;
    }
  }
  return nearest_object;
}

void SemanticExplorer::computePoses(int i){
  //  const ObjectPtr &obj = _semantic_map[i];

  //  _poses[0] = Eigen::Vector3f(obj->position().x()+1.0,obj->position().y(),M_PI);
  //  _poses[1] = Eigen::Vector3f(obj->position().x(),obj->position().y()+1.0,-M_PI_2);
  //  _poses[2] = Eigen::Vector3f(obj->position().x()-1.0,obj->position().y(),0);
  //  _poses[3] = Eigen::Vector3f(obj->position().x(),obj->position().y()-1.0,M_PI_2);

}

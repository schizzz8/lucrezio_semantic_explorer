#include "semantic_explorer.h"

int SemanticExplorer::findNearestObject(){
  int nearest_idx=-1;
  float min_dist = std::numeric_limits<float>::max();
  for(size_t i=0; i<_semantic_map.size(); ++i){
    const ObjectPtr &obj = _semantic_map[i];
    float dist=(obj->position()-_robot_pose.translation()).norm();
    if(dist<min_dist){
      min_dist=dist;
      nearest_idx=i;
    }
  }
  return nearest_idx;
}

void SemanticExplorer::computePoses(int i){
  const ObjectPtr &obj = _semantic_map[i];

  _poses.clear();

  _poses[0] = Eigen::Vector3f(obj->position().x()+1.0,obj->position().y(),M_PI);
  _poses[1] = Eigen::Vector3f(obj->position().x(),obj->position().y()+1.0,-M_PI_2);
  _poses[2] = Eigen::Vector3f(obj->position().x()-1.0,obj->position().y(),0);
  _poses[3] = Eigen::Vector3f(obj->position().x(),obj->position().y()-1.0,M_PI_2);

}

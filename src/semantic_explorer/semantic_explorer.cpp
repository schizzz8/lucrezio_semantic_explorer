#include "semantic_explorer.h"

void serializeRays(const Vector3fPairVector& rays, const std::string& filename);

SemanticExplorer::SemanticExplorer(){
  _camera_pose.setIdentity();
  _objects.clear();
  _processed.clear();
  _N=4;
  _radius=1.0;
}

void SemanticExplorer::setObjects(const ObjectPtrVector& semantic_map){
  for(size_t i=0; i<semantic_map.size(); ++i){
    const ObjectPtr& o = semantic_map[i];
    const std::string& model = o->model();

    if(model == "salt" || model == "milk" || model == "tomato_sauce" || model == "zwieback")
      continue;

    //check if the object has been already processed
    StringVector::iterator it = std::find (_processed.begin(),_processed.end(),model);
    if(it!=_processed.end())
      continue;

    _objects.insert(std::make_pair(model,o));
  }
}

bool SemanticExplorer::findNearestObject(ObjectPtr &nearest_object){
  float min_dist = std::numeric_limits<float>::max();
  bool found=false;
  for(StringObjectPtrMap::iterator it=_objects.begin(); it!=_objects.end(); ++it){
    const ObjectPtr& o = it->second;

    //check if the object has been already processed
    StringVector::iterator itt = std::find (_processed.begin(),_processed.end(),o->model());
    if(itt!=_processed.end()){
      throw std::runtime_error("[SemanticExplorer][findNearestObject]: you're messing up things!");
      continue;
    }

    float dist=(o->position()-_camera_pose.translation()).norm();
    if(dist<min_dist){
      min_dist=dist;
      nearest_object=o;
      found=true;
    }
  }
  return found;
}

Isometry3fVector SemanticExplorer::generateCandidateViews(const ObjectPtr& nearest_object){
  if(!nearest_object)
    throw std::runtime_error("[SemanticExplorer][computePoses]: no nearest object!");

  Isometry3fVector candidate_views;
  for(int i=0; i<_N; i++){
    float alpha=i*(2*M_PI/(float)_N);
    float x=_radius*cos(alpha);
    float y=_radius*sin(alpha);
    float theta=atan2(-y,-x);

    Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
    T.translation() = Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,0.6);
    T.linear() = Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ()).matrix();

    candidate_views.push_back(T);
  }

  return candidate_views;
}

void SemanticExplorer::computeNBV(const Isometry3fVector& candidate_views, const ObjectPtr& nearest_object){
  if(candidate_views.empty())
    throw std::runtime_error("[SemanticExplorer][computeNBV]: no candidate views!");

  //clear queue
  _views = ScoredPoseQueue();

  int unn_max=-1;
  Vector3fPairVector rays;

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
      0.0, 554.25, 240.5,
      0.0,    0.0,   1.0;
  Eigen::Matrix3f inverse_camera_matrix = K.inverse();

  //camera offset
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

  //simulate view
  for(int i=0; i<candidate_views.size(); ++i){

    const Eigen::Isometry3f& T = candidate_views[i];

    //set ray origin to camera pose
    octomap::point3d origin(T.translation().x(),T.translation().y(),T.translation().z());
    std::cerr << "Evaluating view: " << origin << " => ";

    //generate rays
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
    int occ=0,fre=0,unn=0;
    std::vector<octomap::point3d> ray;
    for (int r=0; r<480; r=r+40)
      for (int c=0; c<640; c=c+40){

        //compute ray endpoint
        end=inverse_camera_matrix*Eigen::Vector3f(c,r,1);
        end.normalize();
        end=5*end;
        end=camera_offset*end;
        end=T*end;
        octomap::point3d dir(end.x(),end.y(),end.z());

        //store ray
        rays.push_back(std::make_pair(T.translation(),end));

        //ray casting
        ray.clear();
        if(nearest_object->octree()->computeRay(origin,dir,ray)){
          for(const octomap::point3d voxel : ray){

            if(!nearest_object->inRange(voxel.x(),voxel.y(),voxel.z()))
              continue;

            octomap::OcTreeNode* n = nearest_object->octree()->search(voxel);
            if(n){
              double value = n->getOccupancy();
              if(value>0.5)
                occ++;
              else
                fre++;
              break;
            } else
              unn++;
          }
        }
      }
    std::cerr << "occ: " << occ << " - unn: " << unn << " - fre: " << fre << std::endl;

    if(unn>unn_max){
      unn_max = unn;
      _rays = rays;
    }
    ScoredPose view;
    view.score = unn;
    view.pose = T.translation();
    _views.push(view);
    rays.clear();
  }

  //  std::cerr << "Nearest object occ voxels: " << _nearest_object->occVoxelCloud()->size() << std::endl;
  //  std::cerr << "Nearest object fre voxels: " << _nearest_object->freVoxelCloud()->size() << std::endl;
  //  pcl::io::savePCDFileASCII("occ_cloud.pcd", *_nearest_object->occVoxelCloud());
  //  pcl::io::savePCDFileASCII("fre_cloud.pcd", *_nearest_object->freVoxelCloud());
  //  serializeRays(_rays,"rays.txt");
}

void SemanticExplorer::setProcessed(const ObjectPtr& nearest_object){
  if(!nearest_object)
    throw std::runtime_error("[SemanticExplorer][setProcessed]: no nearest object!");

  StringObjectPtrMap::iterator it = _objects.find(nearest_object->model());
  if(it!=_objects.end()){
    const ObjectPtr& o = it->second;
    _processed.push_back(o->model());
    _objects.erase(it);
  } else
    throw std::runtime_error("[SemanticExplorer][setProcessed]: you're messing up things!");
}

void serializeRays(const Vector3fPairVector& rays, const std::string& filename){
  std::ofstream data;
  data.open(filename);

  for(int i=0; i<rays.size(); ++i){
    const std::pair<Eigen::Vector3f,Eigen::Vector3f>& ray = rays[i];
    const Eigen::Vector3f& first = ray.first;
    const Eigen::Vector3f& second = ray.second;
    data << first.x() << " " << first.y() << " " << first.z() << " ";
    data << second.x() << " " << second.y() << " " << second.z() << std::endl;
  }
  data.close();
}

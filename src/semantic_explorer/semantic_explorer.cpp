#include "semantic_explorer.h"

void serializeRays(const Vector3fPairVector& rays, const std::string& filename);

SemanticExplorer::SemanticExplorer(){
  _camera_pose.setIdentity();
  _objects.clear();
  _processed.clear();
  _N=4;         //Number of NBV candidates
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

//    Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
//    T.translation() = Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,0.6);
//    T.linear() = Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ()).matrix();

    Eigen::Isometry3f T = v2t(Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,theta));

    candidate_views.push_back(T);
  }

  return candidate_views;
}

Isometry3fVector SemanticExplorer::generateCandidateViews_Jose(const ObjectPtr& nearest_object){
  if(!nearest_object){
    throw std::runtime_error("[SemanticExplorer][computePoses]: no nearest object!");
  }
  
  Eigen::Vector3f squaredDistances;
  float OFFSET = 0.1;
  float CLEARANCE = 0.4;
  Isometry3fVector candidate_views;
  squaredDistances[0]=pow(nearest_object->position().x()-(nearest_object->max()[0]+OFFSET),2);
  squaredDistances[1]=pow(nearest_object->position().y()-(nearest_object->max()[1]+OFFSET),2);
  float _radius=sqrt(squaredDistances[0]+squaredDistances[1])+CLEARANCE;

  for(int i=0; i<8; i++){
    float alpha=i*(2*M_PI/((float)8));
    float x=_radius*cos(alpha);
    float y=_radius*sin(alpha);
    float theta=atan2(-y,-x);

    Eigen::Isometry3f T = v2t(Eigen::Vector3f(nearest_object->position().x()+x,nearest_object->position().y()+y,theta));

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
    view.pose = t2v(T);
    _views.push(view);
    rays.clear();
  }

  //  std::cerr << "Nearest object occ voxels: " << _nearest_object->occVoxelCloud()->size() << std::endl;
  //  std::cerr << "Nearest object fre voxels: " << _nearest_object->freVoxelCloud()->size() << std::endl;
  //  pcl::io::savePCDFileASCII("occ_cloud.pcd", *_nearest_object->occVoxelCloud());
  //  pcl::io::savePCDFileASCII("fre_cloud.pcd", *_nearest_object->freVoxelCloud());
  //  serializeRays(_rays,"rays.txt");
}

/*------------------------NBV_Jose------------------------*/

std::vector<int> SemanticExplorer::computeNBV_Jose(const Isometry3fVector& candidate_views, const ObjectPtr& nearest_object){
  if(candidate_views.empty())
    throw std::runtime_error("[SemanticExplorer][computeNBV_Jose]: no candidate views!");

  //clear queue
  _views = ScoredPoseQueue();

  octomap::point3d sensorOrigin(_camera_pose.translation()[0],_camera_pose.translation()[1],_camera_pose.translation()[2]);

  //>>>>>>>>>> Get nearest_object info <<<<<<<<<<
  std::cerr<<"camera at "<<sensorOrigin;
  PointCloud::Ptr cloud ;
  Eigen::Vector4f centroid;
  cloud=nearest_object->cloud();
  Eigen::Vector3f& max=nearest_object->max();
  Eigen::Vector3f& min=nearest_object->min();
  pcl::compute3DCentroid(*cloud,centroid);
  std::cerr << "[SemanticExplorer][computeNBV_Jose]: centroid! "<<centroid(0)<<"  "<<centroid(1)<<"  "<<centroid(2)<< std::endl;
  float cloudCentroid[3]={centroid(0),centroid(1),centroid(2)}; //TODO use centroid and delete cloudCentroid

    //>>>>>>>>>> Create octree <<<<<<<<<<

    /*  Two octrees are needed, "tree" will have all the information
        needed for processing, "cloudAndUnknown" is a simplified OcTree
        containing just the known occupied voxels and unknown voxels.   */

    octomap::Pointcloud scan;       //  Pointcloud will hold the .pcd information
    float MIN_RESOLUTION = 0.02;
    octomap::OcTree tree (MIN_RESOLUTION),cloudAndUnknown (MIN_RESOLUTION);

    for (size_t i = 0; i < cloud->points.size (); ++i){     //  Iterate over the loaded .pcd file

        //  Insert the .pcd cloud points into scan
        scan.push_back(cloud->points[i].x
                ,cloud->points[i].y
                ,cloud->points[i].z);

        //  Insert the .pcd cloud points into the octree cloudAndUnknown
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x
                ,cloud->points[i].y
                ,cloud->points[i].z,true);

        /*  The number 13(random) is saved on each cloud voxel, so that when rayCast
                finds the node I can check if ==13 I found an object voxel  */

        cloudNode->setValue(13);

    }

    /*  InsertPointCloud automatically sets as unknown voxels, the ones behind scan when seeing from sensorOrigin
            the algorithm will look for the unknown volxels which are necesary to compute the NBV. */

    tree.insertPointCloud(scan,sensorOrigin);

    //>>>>>>>>>> Create pointwall as Field of View of the camera <<<<<<<<<<

    /*  Create point wall representing the Field of View of the camera so that raytracing is made
            from the camera center towards each of its points.
            Assuming camera center is in (0,0,0) and the camera direction is towards (1,0,0).
            Kinect has a Field of View of (58.5 x 46.6 degrees) and a Resolution of (320 x 240 px).
            Offline computation results in a pointwall from (1,-0.56,-0.431) to (1,0.56,0.431),
            with 320 points along the Y axes (0.003489 distance between points),
            with 240 points along the Z axes (0.003574 distance between points).    */

    octomap::Pointcloud pointwall;      //  pointwall will represent the sensor FoV in origin coordinates.
    octomap::Pointcloud pointwallOrigin;        //  pointwallOrigin will represent the sensor FoV in origin coordinates,
                                                //      rolled to point towards the cloud centroid
    octomap::Pointcloud pointwallSensor;        //  pointwallSensor will be translated to be in front of the sensor for visualization (check.bt)
    octomap::point3d Point3dwall(1,0,0);    //  each point3d to be inserted into Pointwall

    //  Iterate on each pixel (320x240)
    for(int ii=1;ii<321;ii++){

        for(int iii=1;iii<241;iii++){

            Point3dwall.y()= (-0.560027)+(ii*0.003489);
            Point3dwall.z()= (-0.430668)+(iii*0.003574);
            pointwallOrigin.push_back(Point3dwall);

        }

    }

    pointwall=pointwallOrigin;

    /*  Rotate the pointwall to the camera origin and ortogonal to the vector pointing from
            the camera origin to the centroid, translation is not needed since the function
            castRay works with origin and direction(not end).   */

    octomath::Vector3 Translation(0,0,0),T2(sensorOrigin);
    float roll=atan2(-sensorOrigin.y()+centroid(1),-sensorOrigin.x()+centroid(0));

    octomath::Quaternion Rotation(0,0,roll),R2(0,0,0);       // (Yaw,Pitch,Roll)
    octomap::pose6d RotandTrans(Translation,Rotation),RT2(T2,R2);

    pointwallOrigin.transform(RotandTrans);
    pointwallSensor=pointwallOrigin;
    pointwallSensor.transform(RT2);              //  For visualization PW is translated in front of the sensorOrigin
    octomap::point3d iterator;      //  Helper needed for castRay function

    //>>>>>>>>>> Create background wall to identify known empty volxels <<<<<<<<<<

    /*	A background wall is built leaving empty the shadow of the object, this is
            necesary so that the octree can recognize what area is empty known and
            unknown, otherwise it will assume all tree.writeBinary("check.bt");surroundings of the cloud as unknown.  */

    float alpha;	//	Angle in xy plane from sensorOrigin to each point in Pointwall
    float beta;		//	Elevation angle from sensorOrigin to each point in Pointwall
    float xp, yp, zp;		//	x,y,z coordinates of each point in Pointwall expressed in sensorOrigin coordinates
    float legAdjacentPointWall;		//	Leg adjacent length of a right triangle formed from sensorOrigin to each point in Pointwall
    float legAdjacentBackgroundPoint;		//	Leg adjacent length of a right triangle formed from sensorOrigin to the new background point
    float distance;		//  Distance from the sensorOrigin and the new background point
    octomap::Pointcloud backgroundWall;     //  Pointcloud holding the background wall

    //  distance will be computed so that the wall is always behind the object
    //  distance = 2D_Distance-Centroid-FarthermostPointInBBox + offset + 2D_Distance-sensorOrigin-Centroid 
    float OFFSET=0.2;
    Eigen::Vector3f squaredDistances;
    squaredDistances[0]=pow(centroid(0)-(max[0]+OFFSET),2);
    squaredDistances[1]=pow(centroid(1)-(max[1]+OFFSET),2);

    distance=sqrt(squaredDistances[0]+squaredDistances[1]);

    squaredDistances[0]=pow(centroid(0)-sensorOrigin.x(),2);
    squaredDistances[1]=pow(centroid(1)-sensorOrigin.y(),2);

    distance+=sqrt(squaredDistances[0]+squaredDistances[1]);

    for(int i=0;i<pointwallOrigin.size();i++){

        if(!tree.castRay(sensorOrigin,pointwallOrigin.getPoint(i),iterator)){

            //	Transform pointwall point to sensorOrigin coordinates subtracting sensorOrigin
            xp=-sensorOrigin.x()+pointwallSensor.getPoint(i).x();
            yp=-sensorOrigin.y()+pointwallSensor.getPoint(i).y();
            zp=-sensorOrigin.z()+pointwallSensor.getPoint(i).z();

            //	Get alpha and beta angles
            alpha=atan2(yp,xp);
            legAdjacentPointWall=sqrt((xp*xp)+(yp*yp));
            beta=atan2(zp,legAdjacentPointWall);

            //	Get the new background points and return to global coordinates by adding sensorOrigin
            iterator.z()=sensorOrigin.z()+distance*sin(beta);
            legAdjacentBackgroundPoint=sqrt((distance*distance)-(zp*zp));
            iterator.y()=sensorOrigin.y()+legAdjacentBackgroundPoint*sin(alpha);
            iterator.x()=sensorOrigin.x()+legAdjacentBackgroundPoint*cos(alpha);

            backgroundWall.push_back(iterator);		//	add points to point cloud

        }

    }

    tree.insertPointCloud(backgroundWall,sensorOrigin);     //  Check if i can use other than scan, since it contains the cloud
    //  tree.insertPointCloud(pointwallSensor,sensorOrigin);     //  PW inserted for visualizing the FoV from the sensorOrigin
    tree.writeBinary("check.bt");       //  Visualize "check.bt" with octovis

    //>>>>>>>>>> Search for unknown voxels <<<<<<<<<<

    float RESOLUTION = MIN_RESOLUTION;		//	Search resolution
    std::cerr << "[SemanticExplorer][computeNBV_Jose]: searching for unknown voxels... "<< std::endl;

    //	Iterate over all the bounding box and search for unknown voxels
    for (float ix = min[0]-OFFSET; ix < max[0]+OFFSET; ix += RESOLUTION){

        for (float iy = min[1]-OFFSET; iy < max[1]+OFFSET; iy += RESOLUTION){

            for (float iz = min[2]-OFFSET; iz < max[2]+OFFSET; iz += RESOLUTION){

                if (tree.search(ix,iy,iz)==NULL){		//	If ==NULL it did not find any known (occupied or empty) voxel

                    //	Add a voxel in the empty position in the cloudAndUnknown OcTree
                    iterator.x()=ix;
                    iterator.y()=iy;
                    iterator.z()=iz;

                    octomap::OcTreeNode * unknownCloud=cloudAndUnknown.updateNode(iterator,false);  //  Compose tree needed for NBV computation

                    /*  The number 24(random) is saved on each cloud voxel, so that when rayCast
                            finds the node I can check if ==24 I found an unknown voxel */

                    unknownCloud->setValue(24);

                }

            }

        }

    }

    cloudAndUnknown.writeBinary("check_unknown.bt");       //  Visualize the pointcloud with the unknown voxels

    //>>>>>>>>>> Compute Next Best View candidates <<<<<<<<<<

    /*  The candidates will be computed at a constant distance from the object, all the views
            will be pointing towards the centroid of the cloud. The candidates are computed in Z=0
            by circular tessellation. Further updates will include sphere tessellation for 3D NBV.    */
    std::vector<int> scored_candidate_poses;
    int NBV_CANDIDATENUMBER=8;		//  Number of candidates
    float AngleBetweenCandidate=6.2832/NBV_CANDIDATENUMBER;		//	AngleBetweenCandidate= 360 degrees (2*pi)/ Number of candidates
    float Candidates [NBV_CANDIDATENUMBER][5];		//	Array holding NBV candidates position [x,y,z],roll, and Occlussion Aware VI
    float P_OCCUPATION=0.5;		//	Probability of a random voxel being occupied
    int iNBV=0;     //  Index of the Next Best View in Candidates[i]
    float NBV_DISTANCE=0.4;      //  Closest distance from the NBV candidates to the cloud
    octomap::Pointcloud variablePointwall;      //  variablePointwall hold the FoV of the NBV candidates

    octomap::Pointcloud NBVpointwall;       //  NBVpointwall is a pointcloud holding the Field of View of the NBV candidates

    /*  raytrace from NBVcandidate position to each point in NBVpointwall,
            so this needs to be always behind the cloudAndUnknown voxels    */

    squaredDistances[0]=pow(centroid(0)-(max[0]+OFFSET),2);
    squaredDistances[1]=pow(centroid(1)-(max[1]+OFFSET),2);
    float distanceNBV_centroid=sqrt(squaredDistances[0]+squaredDistances[1]);
    distance=(distanceNBV_centroid*2);
    Point3dwall.x()=distance;

    float yLimit = distance*tan(0.51051);
    float zLimit = distance*tan(0.40666);
    float yPixelDistance = 2*yLimit/320;
    float zPixelDistance = 2*zLimit/240;

    //  Compute NBVpointwall at distance
    for(int ii=1;ii<321;ii++){

        for(int iii=1;iii<241;iii++){

            Point3dwall.y()= (-yLimit)+(ii*yPixelDistance);
            Point3dwall.z()= (-zLimit)+(iii*zPixelDistance);
            NBVpointwall.push_back(Point3dwall);

        }

    }
    std::cerr << "[SemanticExplorer][computeNBV_Jose]: Raytracing... "<< std::endl;
    for(int i=0;i<candidate_views.size();i++){
        const Eigen::Isometry3f& T = candidate_views[i];
        /* 	The Pointcloud representing the FoV of the kinect is rotated and translated
                so that computeRayKeys can raytrace from the NBVcandidate position until
                each of this Pointcloud points.    */
        
        iterator.x()=Candidates[i][0]=T.translation().x();       // [x]
        iterator.y()=Candidates[i][1]=T.translation().y();       // [y]
        iterator.z()=Candidates[i][2]=T.translation().z();                                          // [z]
        Candidates[i][3]=T.linear().eulerAngles(0, 1, 2)[2];
        Candidates[i][4]=0;		//	Set 0 the candidates Occlussion Aware VI
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		//	Vector with the NBVcandidate coordinates
        octomath::Quaternion Rotation2(0,0,Candidates[i][3]);		//	Quaternion containing the roll of NBVcandidate
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);		//	Pose6D contains the pose (rotation and translation) of the NBVcandidate
        variablePointwall=NBVpointwall;		//	Reset variablePointwall
        variablePointwall.transform(RotandTrans2);		//	Change the pose of variablePointwall
	
	// VISUALIZE CANDIDATES AND FIELD OF VIEW

	

        //  Occlussion Aware VI is used as method to compute "how good" is a candidate
        octomap::KeyRay rayBeam; 		//	Contains the position of each voxel in a ray
        int unknownVoxelsInRay=0;		//	Counter of unknown voxels found in a ray
        for(int ii=0;ii<variablePointwall.size();ii++){		//	iterate over all the pointwall
            bool Continue=true;		//	Boolean needed to stop searching when the ray hits a known occupied voxel

            //	Get the position of each voxel in a ray starting from NBVcandidate to each point in variablePointwall
            cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode * node=cloudAndUnknown.search(*it);		//	return the voxel found in such position
                if(node!=NULL){		//	when !=NULL, it found an unknown voxel or a known occupied voxel

                    //	When found a known occupied voxel stop
                    if (node->getValue()==13){

                        Continue=false;

                    }

                    /*	When found an unknown voxel it will add the probability of of being seen from the
                            NBVcandidate position which is given by the P_OCCUPATION to the number of
                            unknown voxels from NBVcandidate to that voxel. */

                    else if(node->getValue()==24){

                        unknownVoxelsInRay++;
                        Candidates[i][4]+=pow(P_OCCUPATION,unknownVoxelsInRay);

                    }

                }

            }

            unknownVoxelsInRay=0;		//	set to 0 after it stops raytracing

        }

        std::cout<< std::endl<<"from (" << Candidates[i][0] << ") ("<< Candidates[i][1] << ") ("<< Candidates[i][2]<< ") Angle: "<< Candidates[i][3]<<" VI: "<<Candidates[i][4];
        ScoredPose view;
        view.score = (int)Candidates[i][4];
        Eigen::Vector3f NBVpos;
        scored_candidate_poses.push_back(Candidates[i][4]);
        NBVpos[0]=Candidates[i][0];
        NBVpos[1]=Candidates[i][1];
        NBVpos[2]=Candidates[i][3];     // just need 2D
        //NBVpos[3]=Candidates[i][3];
        view.pose = NBVpos;
        _views.push(view);

        //	search the index of the biggest VI
        if(Candidates[i][4]>Candidates[iNBV][4]){

            iNBV=i;

        }

    }

    //	Print NBV
    std::cout<< std::endl<<"NEXT BEST VIEW IS (" << Candidates[iNBV][0] << ") ("<< Candidates[iNBV][1] << ") ("<< Candidates[iNBV][2]<<")";
  return scored_candidate_poses;
}

/*-----------------------/NBV_Jose/-----------------------*/

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

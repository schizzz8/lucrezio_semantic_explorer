#include "frontier_detector.h"

using namespace srrg_core;

void FrontierDetector::setup(){
  _resolution = 0.0f;
  _origin.setZero();
  _rows = 0;
  _cols = 0;

  _robot_pose = Eigen::Isometry3f::Identity();
}

void FrontierDetector::init(){
  assert(_resolution != 0 && "[FrontierDetector][init]: Zero grid resolution!");
  assert(_rows != 0 && _cols != 0 && "[FrontierDetector][init]: Image not set");

  _frontier_points.clear();
  _frontier_regions.clear();
  _frontier_centroids.clear();
  _frontier_scored_centroids = ScoredCellQueue();
}

void FrontierDetector::compute(){
  //compute
  computeFrontierPoints();
  computeFrontierRegions();
  computeFrontierCentroids();
  rankFrontierCentroids();
}

void FrontierDetector::computeFrontierPoints(){

  Eigen::Vector2i cell;
  Vector2iVector neighbors;

  for(int r=0; r<_rows; ++r){
    const unsigned char *occupancy_ptr = _occupancy_grid.ptr<unsigned char>(r);
    for(int c=0; c<_cols; ++c, ++occupancy_ptr){
      const unsigned char &occupancy = *occupancy_ptr;

      if(occupancy != Occupancy::FREE)
        continue;

      cell.x() = c;
      cell.y() = r;

      getColoredNeighbors(neighbors,cell,Occupancy::UNKNOWN);

      if(neighbors.empty() || neighbors.size() < 2)
        continue;

      _frontier_points.push_back(cell);
    }
  }
}

void FrontierDetector::computeFrontierRegions(){
  Vector2iList frontiers(_frontier_points.begin(), _frontier_points.end());

  Vector2iList::iterator it;
  for (it = frontiers.begin(); it != frontiers.end(); ++it) {
    Vector2iVector region;
    recurRegion(it, region, frontiers);
    if (region.size() >= _config.size_threshold) {
      _frontier_regions.push_back(region);
    }
    it = frontiers.begin();
  }

  std::cerr << "Detected " << _frontier_regions.size() << " frontier regions" << std::endl;
}

void FrontierDetector::computeFrontierCentroids(){
  for (int i = 0; i < _frontier_regions.size(); ++i) {
    Eigen::Vector2i centroid;
    centroid.setZero();

    for (int j = 0; j <_frontier_regions[i].size(); ++j)
      centroid += _frontier_regions[i][j];

    centroid /= (float)_frontier_regions[i].size();

    _frontier_centroids.push_back(centroid);
  }
}

void FrontierDetector::rankFrontierCentroids(){

  //compute distance map
  IntImage indices_image;
  FloatImage distance_image;
  grayMap2indices(indices_image,_occupancy_grid,60,240);
  indices2distances(distance_image,indices_image,_resolution,_config.radius);

  Eigen::Vector2i robot_position = ((_robot_pose.translation().head(2)-_origin)/_resolution).cast<int>();
  Eigen::Vector2i robot_cell (_rows - robot_position.y(), robot_position.x());
  float robot_orientation = _robot_pose.linear().eulerAngles(0,1,2).z();

  for(const Eigen::Vector2i &frontier_centroid : _frontier_centroids){

    const Eigen::Vector2i diff = frontier_centroid - robot_cell;
    float distance_to_robot = diff.norm();
//    if(distance_to_robot < _config.d2r_threshold)
//      continue;

    float angle_to_robot = std::cos(angleDifference(robot_orientation,std::atan2(diff.y(),diff.x())));
//    if(angle_to_robot < _config.angle_threshold)
//      continue;

    float distance_to_obstacle = std::fabs(distance_image.at<float>(frontier_centroid.y(),frontier_centroid.x()));

    if(distance_to_obstacle < _config.d2o_threshold)
      continue;

    ScoredCell scored_centroid;
    scored_centroid.cell = frontier_centroid;
    float score = distance_to_robot*(1/angle_to_robot)*distance_to_obstacle;

    scored_centroid.score = score;

    _frontier_scored_centroids.push(scored_centroid);

  }
}

void FrontierDetector::getColoredNeighbors(Vector2iVector &neighbors,
                                           const Eigen::Vector2i &cell,
                                           const Occupancy &value){

  neighbors.clear();
  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {

      if (r == 0 && c == 0)
        continue;

      int rr = cell.y()+r;
      int cc = cell.x()+c;

      if ( rr < 0 || rr >= _rows ||
           cc < 0 || cc >= _cols)
        continue;

      if (_occupancy_grid.at<unsigned char>(rr,cc) == value)
        neighbors.push_back(Eigen::Vector2i(cc,rr));

    }
  }
}

void FrontierDetector::recurRegion(const Vector2iList::iterator& frontier_it, Vector2iVector& region, Vector2iList& frontiers) {
  Eigen::Vector2i frontier = *frontier_it;
  region.push_back(frontier);
  frontiers.erase(frontier_it);

  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {

      if (r == 0 && c == 0)
        continue;

      int rr = frontier.y()+r;
      int cc = frontier.x()+c;

      if (rr < 0 || rr >= _rows || cc < 0 || cc >= _cols)
        continue;

      Eigen::Vector2i n(cc,rr);
      Vector2iList::iterator it = std::find(frontiers.begin(), frontiers.end(), n);
      if (it != frontiers.end())
        recurRegion(it, region, frontiers);

    }
  }
}

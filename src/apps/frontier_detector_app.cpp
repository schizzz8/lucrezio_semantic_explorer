#include <iostream>
#include <yaml-cpp/yaml.h>
#include <frontier_detector/frontier_detector.h>

using namespace std;
using namespace srrg_core;

void readMapFromYaml(char *filename);
std::string image_filename;
float resolution;
Eigen::Vector2f origin;

void drawFrontierPoints(RGBImage &image, const Vector2iVector &points);
void drawFrontierRegions(RGBImage &image, const RegionVector &regions);
void drawFrontierScoredCentroids(RGBImage &image, ScoredCellQueue & scored_centroids);

int main(int argc, char **argv){

  readMapFromYaml(argv[1]);

  cv::Mat occupancy_grid;
  occupancy_grid = cv::imread(image_filename,CV_LOAD_IMAGE_UNCHANGED);

  cv::imshow("input",occupancy_grid);

  RGBImage occupancy_rgb;
  cv::cvtColor(occupancy_grid,occupancy_rgb,CV_GRAY2BGR);

  FrontierDetector detector;
  detector.setResolution(resolution);
  detector.setOrigin(origin);
  detector.setMap(occupancy_grid);

  detector.init();

  //compute frontier points
  detector.computeFrontierPoints();
  const Vector2iVector &points = detector.frontierPoints();
  std::cerr << "Detected " << points.size() << " frontiers" << std::endl;

  RGBImage points_image;
  occupancy_rgb.copyTo(points_image);
  drawFrontierPoints(points_image,points);
  cv::imshow("Frontier points",points_image);

  //compute frontier regions
  detector.computeFrontierRegions();
  const RegionVector &regions = detector.frontierRegions();
  std::cerr << "Frontier regions: " << regions.size() << std::endl;

  RGBImage regions_image;
  occupancy_rgb.copyTo(regions_image);
  drawFrontierRegions(regions_image,regions);
  cv::imshow("Frontier regions",regions_image);

  //compute and rank frontier centroids
  detector.computeFrontierCentroids();
  detector.rankFrontierCentroids();
  ScoredCellQueue scored_centroids = detector.frontierScoredCentroids();

  RGBImage scored_centroids_image;
  occupancy_rgb.copyTo(scored_centroids_image);
  drawFrontierScoredCentroids(scored_centroids_image,scored_centroids);
  cv::imshow("Frontier centroids",scored_centroids_image);

  cv::waitKey();

  return 0;
}

void readMapFromYaml(char *filename){
  if(!filename)
    return;

  YAML::Node map = YAML::LoadFile(filename);

  image_filename = map["image"].as<std::string>();
  resolution = map["resolution"].as<float>();
  std::vector<float> std_origin = map["origin"].as<std::vector<float> >();
  origin.x() = std_origin[0];
  origin.y() = std_origin[1];

}

void drawFrontierPoints(RGBImage &image, const Vector2iVector &points){
  for(const Eigen::Vector2i &point : points){
//    cv::Point2i cell(point.y(),point.x());
    cv::Point2i cell(point.x(),point.y());
    cv::circle(image,cell,1,cv::Scalar(0,0,255));
  }
}

void drawFrontierRegions(RGBImage &image, const RegionVector &regions){
  cv::RNG rng(12345);
  for(const Vector2iVector region : regions){
    cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    for(const Eigen::Vector2i &point : region){
//      cv::Point2i cell(point.y(),point.x());
      cv::Point2i cell(point.x(),point.y());
      cv::circle(image,cell,1,color);
    }
  }
}

void drawFrontierScoredCentroids(RGBImage &image, ScoredCellQueue &scored_centroids){
  int count = 0;
  while (!scored_centroids.empty()) {
    ScoredCell centroid = scored_centroids.top();
//    cv::Point2i cell(centroid.cell.y(),centroid.cell.x());
    cv::Point2i cell(centroid.cell.x(),centroid.cell.y());
    cv::circle(image,cell,4,cv::Scalar(255,0,0),2);
    float score = centroid.score;
    std::ostringstream ss;
    ss << count;
    cv::putText(image, ss.str(), cell, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
    std::cerr << "Centroid " << count << ": " << score << std::endl;
    count++;
    scored_centroids.pop();
  }
}

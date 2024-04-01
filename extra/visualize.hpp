/**
 * visualize for ov2slam on linux
*/

#pragma once

#include <vector>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "ov2slam.hpp"


class Visualize {
public:

  Visualize(std::shared_ptr<SlamManager> pslam);

  void run();

  void drawCamrea();
  void updateCamrea();

  void drawMapPoints();
  void drawKeyFrames();
  void drawGraph();
  
  std::shared_ptr<SlamManager> pslam_;
  pangolin::OpenGlMatrix Twc_;
  std::vector<Eigen::Vector3d> vpos_;

  bool bexit_required_ = false; 

};

/**
*    This file is part of OV²SLAM.
*    
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/
#pragma once


#include <iostream>
#include <string>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <sophus/se3.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "profiler.hpp"

class Options {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Options() {}
    
    Options(const cv::FileStorage &fsSettings);

    void reset();

    //=====================================================
    // Variables relative to the current state of the SLAM
    //=====================================================

    bool blocalba_is_on_ = false;
    bool blc_is_on_ = false;
    bool bvision_init_ = false;
    bool breset_req_ = false;
    bool bforce_realtime_ = false;

    //=====================================================
    // Variables relative to the setup used for the SLAM
    //=====================================================

    // Calibration parameters (TODO: Get Ready to store all of these in a vector to handle N camera)
    std::string cam_left_topic_, cam_right_topic_;
    std::string cam_left_model_, cam_right_model_;

    double fxl_, fyl_, cxl_, cyl_;
    double k1l_, k2l_, p1l_, p2l_;

    double fxr_, fyr_, cxr_, cyr_;
    double k1r_, k2r_, p1r_, p2r_;

    double img_left_w_, img_left_h_;
    double img_right_w_, img_right_h_;

    // Extrinsic parameters
    Sophus::SE3d T_left_right_;

    // SLAM settings
    bool debug_ = false;
    bool log_timings_ = false;

    bool mono_ = true; 
    bool stereo_ = false;

    bool slam_mode_ = true;

    bool buse_loop_closer_ = false;
    int lckfid_ = -1;

    float finit_parallax_ = 20;
    
    bool bdo_stereo_rect_ = false;
    double alpha_ = 0;

    bool bdo_undist_ = false;

    // Keypoints Extraction
    bool use_fast_ = true;
    bool use_shi_tomasi_ = false;
    bool use_singlescale_detector_ = false;
    bool use_brief_ = false;
    
    int nfast_th_ = 10;
    int nbmaxkps_;
    int nmaxdist_ = 45;
    double dmaxquality_ = 0.001;

    // Image Processing
    bool use_clahe_ = false;
    float fclahe_val_ = 3;

    // KLT Parameters
    bool do_klt_ = true, klt_use_prior_ = true;
    bool btrack_keyframetoframe_ = false;
    int nklt_win_size_ = 9;
    int nklt_pyr_lvl_ = 3;
    cv::Size klt_win_size_;

    float fmax_fbklt_dist_ = 0.5;
    int nmax_iter_ = 30;
    float fmax_px_precision_ = 0.01;

    int nklt_err_ = 30;

    // Matching th.
    bool bdo_track_localmap_ = true;
    
    float fmax_desc_dist_ = 0.2;
    float fmax_proj_pxdist_ = 2;

    // Error thresholds
    bool doepipolar_ = true;
    bool dop3p_ = false;
    bool bdo_random = true; // RANDOMIZE RANSAC?
    float fransac_err_ = 3;
    int nransac_iter_ = 100;
    float fepi_th_;

    float fmax_reproj_err_ = 3;
    bool buse_inv_depth_ = 1;

    // Bundle Adjustment Parameters
    // (mostly related to Ceres options)
    float robust_mono_th_ = 5.9915;
    float robust_stereo_th_;

    bool use_sparse_schur_ = true; // If False, Dense Schur used
    bool use_dogleg_ = false; // If False, Lev.-Marq. used
    bool use_subspace_dogleg_ = false; // If False, Powell's trad. Dogleg used
    bool use_nonmonotic_step_ = false;

    // Estimator parameters
    bool apply_l2_after_robust_ = true; // If true, a L2 optim is applied to refine the results from robust cost function

    int nmin_covscore_ = 25; // Number of common observations req. for opt. a KF in localBA

    // Map Filtering parameters
    float fkf_filtering_ratio_ = 0.9;

    // Final BA
    bool do_full_ba_ = false;
};

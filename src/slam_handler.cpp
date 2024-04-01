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

#include <chrono>
#include <opencv2/highgui.hpp>

#include "vo_handler.hpp"


SlamHandler::SlamHandler(std::shared_ptr<Options> poptions)
    : poptions_(poptions)
{
    setupCalibration();

    if( poptions_->stereo_ && poptions_->bdo_stereo_rect_ ) {
        // std::cout << "\n SetupStereoCalibration()\n";
        setupStereoCalibration();
    }
    else if( poptions_->mono_ && poptions_->bdo_stereo_rect_ ) {
        poptions_->bdo_stereo_rect_ = false;
    }

    // If no stereo rectification required (i.e. mono config or 
    // stereo w/o rectification) and image undistortion required
    if( !poptions_->bdo_stereo_rect_ && poptions_->bdo_undist_ ) {
        std::cout << "\n Setup Image Undistortion\n";
        pcalib_model_left_->setUndistMap(poptions_->alpha_);
        if( poptions_->stereo_ )
            pcalib_model_right_->setUndistMap(poptions_->alpha_);
    }

    if( poptions_->mono_ ) {
        pcurframe_.reset( new Frame(pcalib_model_left_, poptions_->nmaxdist_) );
    } else if( poptions_->stereo_ ) {
        pcurframe_.reset( new Frame(pcalib_model_left_, pcalib_model_right_, poptions_->nmaxdist_) );
    } else {
        std::cerr << "\n\n=====================================================\n\n";
        std::cerr << "\t\t YOU MUST CHOOSE BETWEEN MONO / STEREO (and soon RGBD)\n";
        std::cerr << "\n\n=====================================================\n\n";
    }

    // Create all objects to be used within OV²SLAM
    // =============================================
    int tilesize = 50;
    cv::Size clahe_tiles(pcalib_model_left_->img_w_ / tilesize
                        , pcalib_model_left_->img_h_ / tilesize);
                        
    cv::Ptr<cv::CLAHE> pclahe = cv::createCLAHE(poptions_->fclahe_val_, clahe_tiles);

    pfeatextract_.reset( new FeatureExtractor(
                                poptions_->nbmaxkps_, poptions_->nmaxdist_, 
                                poptions_->dmaxquality_, poptions_->nfast_th_
                            ) 
                        );

    ptracker_.reset( new FeatureTracker(poptions_->nmax_iter_, 
                            poptions_->fmax_px_precision_, pclahe
                        )
                    );

    // Map Manager will handle Keyframes / MapPoints
    pmap_.reset( new MapManager(poptions_, pcurframe_, pfeatextract_, ptracker_) );

    // Visual Front-End processes every incoming frames 
    pvisualfrontend_.reset( new VisualFrontEnd(poptions_, pcurframe_, 
                                    pmap_, ptracker_
                                )
                            );

    // (i.e. triangulation, local map tracking, BA, LC)
    pmapper_.reset( new Mapper(poptions_, pmap_, pcurframe_) );
}

void SlamHandler::run()
{
    // std::cout << "\nOV²SLAM is ready to process incoming images!\n";

    bis_on_ = true;

    cv::Mat img_left, img_right;

    double time = -1.; // Current image timestamp
    double cam_delay = -1.; // Delay between two successive images
    double last_img_time = -1.; // Last received image time

    // Main SLAM loop
    while( !bexit_required_ ) {

        // 0. Get New Images
        // =============================================
        if( getNewImage(img_left, img_right, time) )
        {
            // Update current frame
            frame_id_++;
            pcurframe_->updateFrame(frame_id_, time);

            // Update cam delay for automatic exit
            double time_now_ms = (double)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();
            if( frame_id_ > 0 ) {
                cam_delay = time_now_ms / 1e3 - last_img_time;
                last_img_time += cam_delay;
            } else {
                last_img_time = time_now_ms / 1e3;
            }

            // 1. Send images to the FrontEnd
            // =============================================

            bool is_kf_req = pvisualfrontend_->visualTracking(img_left, time);

            if( poptions_->breset_req_ ) {
                reset();
                continue;
            }

            // 2. Create new KF if req. / Send new KF to Mapper
            // ================================================
            if( is_kf_req ) 
            {
                // if( poptions_->debug_ )
                //     std::cout << "\n \t >>> [SLAM Node] New Keyframe send to Back-End\n";

                if( poptions_->stereo_ ) 
                {
                    Keyframe kf(
                        pcurframe_->kfid_, 
                        img_left,
                        img_right,
                        pvisualfrontend_->cur_pyr_
                        );

                    pmapper_->addNewKf(kf);
                } 
                else if( poptions_->mono_ ) 
                {
                    Keyframe kf(pcurframe_->kfid_, img_left);
                    pmapper_->addNewKf(kf);
                }

            }

            // if( poptions_->debug_ || poptions_->log_timings_ )
            //     std::cout << Profiler::getInstance().displayTimeLogs() << std::endl;

        } 
        else {

            // 3. Check if we are done with a sequence!
            // ========================================
            bool c1 = cam_delay > 0;
            double time_now_ms = (double)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();
            bool c2 = ( time_now_ms / 1e3 - last_img_time ) > 100. * cam_delay;
            bool c3 = !bnew_img_available_;

            if( c1 && c2 && c3 )
            {
                bexit_required_ = true;

                // Warn threads to stop and then save the results only in this case of 
                // automatic stop because end of sequence reached 
                // (avoid wasting time when forcing stop by CTRL+C)
                pmapper_->bexit_required_ = true;

                writeResults();

            }
            else {
                std::chrono::milliseconds dura(1);
            }
        }
    }

    // std::cout << "\nOV²SLAM is stopping!\n";

    bis_on_ = false;
}

void SlamHandler::addNewMonoImage(const double time, cv::Mat &im0)
{
    if( poptions_->bdo_undist_ ) {
        pcalib_model_left_->rectifyImage(im0, im0);
    }

    std::lock_guard<std::mutex> lock(img_mutex_);
    qimg_left_.push(im0);
    qimg_time_.push(time);

    bnew_img_available_ = true;
    run();
}

void SlamHandler::addNewStereoImages(const double time, cv::Mat &im0, cv::Mat &im1) 
{
    if( poptions_->bdo_stereo_rect_ || poptions_->bdo_undist_ ) {
        pcalib_model_left_->rectifyImage(im0, im0);
        pcalib_model_right_->rectifyImage(im1, im1);
    }

    std::lock_guard<std::mutex> lock(img_mutex_);
    qimg_left_.push(im0);
    qimg_right_.push(im1);
    qimg_time_.push(time);

    bnew_img_available_ = true;
    run();
}

bool SlamHandler::getNewImage(cv::Mat &iml, cv::Mat &imr, double &time)
{
    std::lock_guard<std::mutex> lock(img_mutex_);

    if( !bnew_img_available_ ) {
        return false;
    }

    int k = 0;

    do {
        k++;

        iml = qimg_left_.front();
        qimg_left_.pop();

        time = qimg_time_.front();
        qimg_time_.pop();
        
        if( poptions_->stereo_ ) {
            imr = qimg_right_.front();
            qimg_right_.pop();
        }

        if( !poptions_->bforce_realtime_ )
            break;

    } while( !qimg_left_.empty() );

    if( k > 1 ) {    
        // if( poptions_->debug_ )
        //     std::cout << "\n SLAM is late!  Skipped " << k-1 << " frames...\n";
    }
    
    if( qimg_left_.empty() ) {
        bnew_img_available_ = false;
    }

    return true;
}

void SlamHandler::setupCalibration()
{
    pcalib_model_left_.reset( 
                new CameraCalibration(
                        poptions_->cam_left_model_, 
                        poptions_->fxl_, poptions_->fyl_, 
                        poptions_->cxl_, poptions_->cyl_,
                        poptions_->k1l_, poptions_->k2l_, 
                        poptions_->p1l_, poptions_->p2l_,
                        poptions_->img_left_w_, 
                        poptions_->img_left_h_
                        ) 
                    );

    if( poptions_->stereo_ ) 
    {
        pcalib_model_right_.reset( 
                    new CameraCalibration(
                            poptions_->cam_right_model_, 
                            poptions_->fxr_, poptions_->fyr_, 
                            poptions_->cxr_, poptions_->cyr_,
                            poptions_->k1r_, poptions_->k2r_, 
                            poptions_->p1r_, poptions_->p2r_,
                            poptions_->img_right_w_, 
                            poptions_->img_right_h_
                            ) 
                        );
        
        // TODO: Change this and directly add the extrinsic parameters within the 
        // constructor (maybe set default parameters on extrinsic with identity / zero)
        pcalib_model_right_->setupExtrinsic(poptions_->T_left_right_);
    }
}

void SlamHandler::setupStereoCalibration()
{
    // Apply stereorectify and setup the calibration models
    cv::Mat Rl, Rr, Pl, Pr, Q;

    cv::Rect rectleft, rectright;

    if( pcalib_model_left_->model_ != pcalib_model_right_->model_ )
    {
        std::cerr << "\n Left and Right cam have different distortion model.  Cannot use stereo rectifcation!\n";
        return;
    }

    if( cv::countNonZero(pcalib_model_left_->Dcv_) == 0 && 
        cv::countNonZero(pcalib_model_right_->Dcv_) == 0 &&
        pcalib_model_right_->Tc0ci_.rotationMatrix().isIdentity(1.e-5) )
    {
        std::cout << "\n No distorsion and R_left_right = I3x3 / NO rectif to apply!";
        return;
    }

    if( pcalib_model_left_->model_ == CameraCalibration::Pinhole )
    {
        cv::stereoRectify(
                pcalib_model_left_->Kcv_, pcalib_model_left_->Dcv_,
                pcalib_model_right_->Kcv_, pcalib_model_right_->Dcv_,
                pcalib_model_left_->img_size_, 
                pcalib_model_right_->Rcv_cic0_, 
                pcalib_model_right_->tcv_cic0_,
                Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 
                poptions_->alpha_,
                pcalib_model_left_->img_size_, 
                &rectleft, &rectright
                );
    }
    else 
    {
        cv::fisheye::stereoRectify(
                pcalib_model_left_->Kcv_, pcalib_model_left_->Dcv_,
                pcalib_model_right_->Kcv_, pcalib_model_right_->Dcv_,
                pcalib_model_left_->img_size_, 
                pcalib_model_right_->Rcv_cic0_, 
                pcalib_model_right_->tcv_cic0_,
                Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 
                pcalib_model_left_->img_size_, 
                poptions_->alpha_
                );
        
        rectleft = cv::Rect(0, 0, pcalib_model_left_->img_w_, pcalib_model_left_->img_h_);
        rectright = cv::Rect(0, 0, pcalib_model_right_->img_w_, pcalib_model_right_->img_h_);
    }

    // std::cout << "\n Alpha : " << poptions_->alpha_;

    // std::cout << "\n Kl : \n" << pcalib_model_left_->Kcv_;
    // std::cout << "\n Kr : \n" << pcalib_model_right_->Kcv_;

    // std::cout << "\n Dl : \n" << pcalib_model_left_->Dcv_;
    // std::cout << "\n Dr : \n" << pcalib_model_right_->Dcv_;

    // std::cout << "\n Rl : \n" << Rl;
    // std::cout << "\n Rr : \n" << Rr;
    
    // std::cout << "\n Pl : \n" << Pl;
    // std::cout << "\n Pr : \n" << Pr;

    // % OpenCV can handle left-right or up-down camera arrangements
    // isVerticalStereo = abs(RCT.P2(2,4)) > abs(RCT.P2(1,4));

    pcalib_model_left_->setUndistStereoMap(Rl, Pl, rectleft);
    pcalib_model_right_->setUndistStereoMap(Rr, Pr, rectright);

    // SLAM state keeps track of the initial intrinsic
    // parameters (perhaps to be used for optim...)
    poptions_->fxl_ = pcalib_model_left_->fx_;
    poptions_->fyl_ = pcalib_model_left_->fy_;
    poptions_->cxl_ = pcalib_model_left_->cx_;
    poptions_->cyl_ = pcalib_model_left_->cy_;

    poptions_->fxr_ = pcalib_model_right_->fx_;
    poptions_->fyr_ = pcalib_model_right_->fy_;
    poptions_->cxr_ = pcalib_model_right_->cx_;
    poptions_->cyr_ = pcalib_model_right_->cy_;
}

void SlamHandler::reset()
{
    pcurframe_->reset();
    pvisualfrontend_->reset();
    pmap_->reset();
    pmapper_->reset();

    poptions_->reset();
    Logger::reset();

    frame_id_ = -1;

    std::lock_guard<std::mutex> lock(img_mutex_);
    
    qimg_left_ = std::queue<cv::Mat>(); 
    qimg_right_ = std::queue<cv::Mat>();
    qimg_time_ = std::queue<double>();

    bnew_img_available_ = false;
}


// ==========================
//   Visualization functions
// ==========================

cv::Mat SlamHandler::visualFrame()
{
    // // Display keypoints
    // cv::Mat img = cv::Mat(poptions_->img_left_h_, poptions_->img_left_w_, CV_8UC3);
    // cv::Mat img_clone;

    // {
    //     std::lock_guard<std::mutex> lock(pvisualfrontend_->img_mutex_);
    //     img_clone = pvisualfrontend_->cur_img_.clone();
    // }
    
    // std::stringstream ss;

    // if(! img_clone.empty()) {
    //     cv::cvtColor(img_clone, img, CV_GRAY2RGB);
    //     for( const auto &kp : pcurframe_->getKeypoints() ) {
    //         cv::Scalar col = kp.is_retracked_ ? ( kp.is3d_ ? cv::Scalar(0,255,0) : cv::Scalar(235, 235, 52) )
    //             : (kp.is3d_ ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255));
    //         cv::circle(img, kp.px_, 4, col, -1);
    //     }

    //     if( poptions_->breset_req_ ) ss << "RESET REQUIRED";
    //     else ss << (poptions_->slam_mode_ ? " SLAM ON  |  " : " SLAM OFF |  ") 
    //             << "KEY FRAMES : " << (pmap_->map_pkfs_.size()) <<", MAP POINTS : " << (pmap_->map_plms_.size());
    // } else {
    //     ss << "WAITING FOR IMAGES";
    // }

    // int baseline = 0;
    // cv::Size textSize = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
    
    // cv::Mat img_text = cv::Mat(img.rows + textSize.height + 10, img.cols, img.type());
    // img.copyTo(img_text.rowRange(0, img.rows).colRange(0, img.cols));
    // img_text.rowRange(img.rows, img_text.rows) =
    //     cv::Mat::zeros(textSize.height + 10, img.cols, img.type());
    // cv::putText(img_text, ss.str(), cv::Point(5, img_text.rows - 5),
    //             cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);

    // return img_text;
}
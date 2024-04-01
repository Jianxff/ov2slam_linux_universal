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


#include "mapper.hpp"
#include "opencv2/video/tracking.hpp"

Mapper::Mapper(std::shared_ptr<Options> poptions, std::shared_ptr<MapManager> pmap, 
            std::shared_ptr<Frame> pframe)
    : poptions_(poptions), pmap_(pmap), pcurframe_(pframe)
    , pestimator_( new Estimator(poptions_, pmap_) )
{

}

void Mapper::run()
{
    Keyframe kf;
    if( getNewKf(kf) ) 
    {
        std::shared_ptr<Frame> pnewkf = pmap_->getKeyframe(kf.kfid_);
        assert( pnewkf );

        if( poptions_->stereo_ ) 
        {

            cv::Mat imright;
            if( poptions_->use_clahe_ ) {
                pmap_->ptracker_->pclahe_->apply(kf.imrightraw_, imright);
            } else {
                imright = kf.imrightraw_;
            }
            std::vector<cv::Mat> vpyr_imright;
            cv::buildOpticalFlowPyramid(imright, vpyr_imright, poptions_->klt_win_size_, poptions_->nklt_pyr_lvl_);

            pmap_->stereoMatching(*pnewkf, kf.vpyr_imleft_, vpyr_imright);
            
            if( pnewkf->nb2dkps_ > 0 && pnewkf->nb_stereo_kps_ > 0 ) {

                std::lock_guard<std::mutex> lock(pmap_->map_mutex_);

                triangulateStereo(*pnewkf);
            }
        }

        // Triangulate temporal
        if( pnewkf->nb2dkps_ > 0 && pnewkf->kfid_ > 0 ) 
        {
            std::lock_guard<std::mutex> lock(pmap_->map_mutex_);
            
            triangulateTemporal(*pnewkf);
        }

        // If Mono mode, check if reset is required
        if( poptions_->mono_ && poptions_->bvision_init_ ) 
        {
            if( kf.kfid_ == 1 && pnewkf->nb3dkps_ < 30 ) {
                poptions_->breset_req_ = true;
                reset();
                continue;
            } 
            else if( kf.kfid_ < 10 && pnewkf->nb3dkps_ < 3 ) {
                poptions_->breset_req_ = true;
                reset();
                continue;
            }
        }

        // Update the MPs and the covisilbe graph between KFs
        // (done here for real-time performance reason)
        pmap_->updateFrameCovisibility(*pnewkf);

        // Dirty but useful for visualization
        pcurframe_->map_covkfs_ = pnewkf->map_covkfs_;

        if( poptions_->use_brief_ && kf.kfid_ > 0 
            && !bnewkfavailable_ ) 
        {
            if( poptions_->bdo_track_localmap_ )
            {
                matchingToLocalMap(*pnewkf);
            }
        }

        // Send new KF to estimator for BA
        pestimator_->addNewKf(pnewkf);
        pestimator_->run();
    }
}


void Mapper::triangulateTemporal(Frame &frame)
{
    // if( poptions_->debug_ || poptions_->log_timings_ )
    //     Profiler::Start("1.KF_TriangulateTemporal");

    // Get New KF kps / pose
    std::vector<Keypoint> vkps = frame.getKeypoints2d();

    Sophus::SE3d Twcj = frame.getTwc();

    if( vkps.empty() ) {
        // if( poptions_->debug_ )
        //     std::cout << "\n \t >>> No kps to temporal triangulate...\n";
        return;
    }

    // Setup triangulatation for OpenGV-based mapping
    size_t nbkps = vkps.size();

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vleftbvs, vrightbvs;
    vleftbvs.reserve(nbkps);
    vrightbvs.reserve(nbkps);

    // Init a pkf object that will point to the prev KF to use
    // for triangulation
    std::shared_ptr<Frame> pkf;
    pkf.reset( new Frame() );
    pkf->kfid_ = -1;

    // Relative motions between new KF and prev. KFs
    int relkfid = -1;
    Sophus::SE3d Tcicj, Tcjci;
    Eigen::Matrix3d Rcicj;

    // New 3D MPs projections
    cv::Point2f left_px_proj, right_px_proj;
    float ldist, rdist;
    Eigen::Vector3d left_pt, right_pt, wpt;

    int good = 0, candidates = 0;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vwpts;
    std::vector<int> vlmids;
    vwpts.reserve(nbkps);
    vlmids.reserve(nbkps);

    // We go through all the 2D kps in new KF
    for( size_t i = 0 ; i < nbkps ; i++ )
    {
        // Get the related MP and check if it is ready 
        // to be triangulated 
        std::shared_ptr<MapPoint> plm = pmap_->getMapPoint(vkps.at(i).lmid_);

        if( plm == nullptr ) {
            pmap_->removeMapPointObs(vkps.at(i).lmid_, frame.kfid_);
            continue;
        }

        // If MP is already 3D continue (should not happen)
        if( plm->is3d_ ) {
            continue;
        }

        // Get the set of KFs sharing observation of this 2D MP
        std::set<int> co_kf_ids = plm->getKfObsSet();

        // Continue if new KF is the only one observing it
        if( co_kf_ids.size() < 2 ) {
            continue;
        }

        int kfid = *co_kf_ids.begin();

        if( frame.kfid_ == kfid ) {
            continue;
        }

        // Get the 1st KF observation of the related MP
        pkf = pmap_->getKeyframe(kfid);
        
        if( pkf == nullptr ) {
            continue;
        }

        // Compute relative motion between new KF and selected KF
        // (only if req.)
        if( relkfid != kfid ) {
            Sophus::SE3d Tciw = pkf->getTcw();
            Tcicj = Tciw * Twcj;
            Tcjci = Tcicj.inverse();
            Rcicj = Tcicj.rotationMatrix();

            relkfid = kfid;
        }

        // If no motion between both KF, skip
        if( poptions_->stereo_ && Tcicj.translation().norm() < 0.01 ) {
            continue;
        }
        
        // Get obs kp
        Keypoint kfkp = pkf->getKeypointById(vkps.at(i).lmid_);
        if( kfkp.lmid_ != vkps.at(i).lmid_ ) {
            continue;
        }

        // Check rotation-compensated parallax
        cv::Point2f rotpx = frame.projCamToImage(Rcicj * vkps.at(i).bv_);
        double parallax = cv::norm(kfkp.unpx_ - rotpx);

        candidates++;

        // Compute 3D pos and check if its good or not
        left_pt = computeTriangulation(Tcicj, kfkp.bv_, vkps.at(i).bv_);

        // Project into right cam (new KF)
        right_pt = Tcjci * left_pt;

        // Ensure that the 3D MP is in front of both camera
        if( left_pt.z() < 0.1 || right_pt.z() < 0.1 ) {
            if( parallax > 20. ) {
                pmap_->removeMapPointObs(kfkp.lmid_, frame.kfid_);
            }
            continue;
        }

        // Remove MP with high reprojection error
        left_px_proj = pkf->projCamToImage(left_pt);
        right_px_proj = frame.projCamToImage(right_pt);
        ldist = cv::norm(left_px_proj - kfkp.unpx_);
        rdist = cv::norm(right_px_proj - vkps.at(i).unpx_);

        if( ldist > poptions_->fmax_reproj_err_ 
            || rdist > poptions_->fmax_reproj_err_ ) {
            if( parallax > 20. ) {
                pmap_->removeMapPointObs(kfkp.lmid_, frame.kfid_);
            }
            continue;
        }

        // The 3D pos is good, update SLAM MP and related KF / Frame
        wpt = pkf->projCamToWorld(left_pt);
        pmap_->updateMapPoint(vkps.at(i).lmid_, wpt, 1./left_pt.z());

        good++;
    }

    // if( poptions_->debug_ )
    //     std::cout << "\n \t >>> Temporal Mapping : " << good << " 3D MPs out of " 
    //         << candidates << " kps !\n";

    // if( poptions_->debug_ || poptions_->log_timings_ )
    //     Profiler::StopAndDisplay(poptions_->debug_, "1.KF_TriangulateTemporal");
}

void Mapper::triangulateStereo(Frame &frame)
{
    // if( poptions_->debug_ || poptions_->log_timings_ )
    //     Profiler::Start("1.KF_TriangulateStereo");

    // INIT STEREO TRIANGULATE
    std::vector<Keypoint> vkps;

    // Get the new KF stereo kps
    vkps = frame.getKeypointsStereo();

    if( vkps.empty() ) {
        // if( poptions_->debug_ )
        //     std::cout << "\n \t >>> No kps to stereo triangulate...\n";
        return;
    }

    // Store the stereo kps along with their idx
    std::vector<int> vstereoidx;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vleftbvs, vrightbvs;

    size_t nbkps = vkps.size();

    vleftbvs.reserve(nbkps);
    vrightbvs.reserve(nbkps);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vwpts;
    std::vector<int> vlmids;
    vwpts.reserve(nbkps);
    vlmids.reserve(nbkps);

    // Get the extrinsic transformation
    Sophus::SE3d Tlr = frame.pcalib_rightcam_->getExtrinsic();
    Sophus::SE3d Trl = Tlr.inverse();

    for( size_t i = 0 ; i < nbkps ; i++ )
    {
        if( !vkps.at(i).is3d_ && vkps.at(i).is_stereo_ )
        {
            vstereoidx.push_back(i);
            vleftbvs.push_back(vkps.at(i).bv_);
            vrightbvs.push_back(vkps.at(i).rbv_);
        }
    }

    if( vstereoidx.empty() ) {
        return;
    }

    size_t nbstereo = vstereoidx.size();

    cv::Point2f left_px_proj, right_px_proj;
    float ldist, rdist;
    Eigen::Vector3d left_pt, right_pt, wpt;

    int kpidx;

    int good = 0;

    // For each stereo kp
    for( size_t i = 0 ; i < nbstereo ; i++ ) 
    {
        kpidx = vstereoidx.at(i);

        if( poptions_->bdo_stereo_rect_ ) {
            float disp = vkps.at(kpidx).unpx_.x - vkps.at(kpidx).runpx_.x;

            if( disp < 0. ) {
                frame.removeStereoKeypointById(vkps.at(kpidx).lmid_);
                continue;
            }

            float z = frame.pcalib_leftcam_->fx_ * frame.pcalib_rightcam_->Tcic0_.translation().norm() / fabs(disp);

            left_pt << vkps.at(kpidx).unpx_.x, vkps.at(kpidx).unpx_.y, 1.;
            left_pt = z * frame.pcalib_leftcam_->iK_ * left_pt.eval();
        } else {
            // Triangulate in left cam frame
            left_pt = computeTriangulation(Tlr, vleftbvs.at(i), vrightbvs.at(i));
        }

        // Project into right cam frame
        right_pt = Trl * left_pt;

        if( left_pt.z() < 0.1 || right_pt.z() < 0.1 ) {
            frame.removeStereoKeypointById(vkps.at(kpidx).lmid_);
            continue;
        }

        // Remove MP with high reprojection error
        left_px_proj = frame.projCamToImage(left_pt);
        right_px_proj = frame.projCamToRightImage(left_pt);
        ldist = cv::norm(left_px_proj - vkps.at(kpidx).unpx_);
        rdist = cv::norm(right_px_proj - vkps.at(kpidx).runpx_);

        if( ldist > poptions_->fmax_reproj_err_
            || rdist > poptions_->fmax_reproj_err_ ) {
            frame.removeStereoKeypointById(vkps.at(kpidx).lmid_);
            continue;
        }

        // Project MP in world frame
        wpt = frame.projCamToWorld(left_pt);

        pmap_->updateMapPoint(vkps.at(kpidx).lmid_, wpt, 1./left_pt.z());

        good++;
    }

    // if( poptions_->debug_ )
    //     std::cout << "\n \t >>> Stereo Mapping : " << good << " 3D MPs out of " 
    //         << nbstereo << " kps !\n";

    // if( poptions_->debug_ || poptions_->log_timings_ )
    //     Profiler::StopAndDisplay(poptions_->debug_, "1.KF_TriangulateStereo");
}

inline Eigen::Vector3d Mapper::computeTriangulation(const Sophus::SE3d &T, const Eigen::Vector3d &bvl, const Eigen::Vector3d &bvr)
{
    // OpenGV Triangulate
    return MultiViewGeometry::triangulate(T, bvl, bvr);
}

bool Mapper::matchingToLocalMap(Frame &frame)
{
    // if( poptions_->debug_ || poptions_->log_timings_ )
    //     Profiler::Start("1.KF_MatchingToLocalMap");

    // Maximum number of MPs to track
    const size_t nmax_localplms = poptions_->nbmaxkps_ * 10;

    // If room for more kps, get the local map  of the oldest co KF
    // and add it to the set of MPs to search for
    auto cov_map = frame.getCovisibleKfMap();

    if( frame.set_local_mapids_.size() < nmax_localplms ) 
    {
        int kfid = cov_map.begin()->first;
        auto pkf = pmap_->getKeyframe(kfid);
        while( pkf == nullptr  && kfid > 0 && !bnewkfavailable_ ) {
            kfid--;
            pkf = pmap_->getKeyframe(kfid);
        }

        // Skip if no time
        if( bnewkfavailable_ ) {
            return false;
        }
        
        if( pkf != nullptr ) {
            frame.set_local_mapids_.insert( pkf->set_local_mapids_.begin(), pkf->set_local_mapids_.end() );
        }

        // If still far not enough, go for another round
        if( pkf->kfid_ > 0 && frame.set_local_mapids_.size() < 0.5 * nmax_localplms )
        {
            pkf = pmap_->getKeyframe(pkf->kfid_);
            while( pkf == nullptr  && kfid > 0 && !bnewkfavailable_ ) {
                kfid--;
                pkf = pmap_->getKeyframe(kfid);
            }

            // Skip if no time
            if( bnewkfavailable_ ) {
                return false;
            }
            
            if( pkf != nullptr ) {
                frame.set_local_mapids_.insert( pkf->set_local_mapids_.begin(), pkf->set_local_mapids_.end() );
            }
        }
    }

    // Skip if no time
    if( bnewkfavailable_ ) {
        return false;
    }

    // if( poptions_->debug_ )
    //     std::cout << "\n \t>>> matchToLocalMap() --> Number of local MPs selected : " 
    //         << frame.set_local_mapids_.size() << "\n";

    // Track local map
    std::map<int,int> map_previd_newid = matchToMap(
                                            frame, poptions_->fmax_proj_pxdist_, 
                                            poptions_->fmax_desc_dist_, frame.set_local_mapids_
                                            );

    size_t nbmatches = map_previd_newid.size();

    // if( poptions_->debug_ )
    //     std::cout << "\n \t>>> matchToLocalMap() --> Match To Local Map found #" 
    //         << nbmatches << " matches \n"; 

    // Return if no matches
    if( nbmatches == 0 ) {
        return false;
    }

    mergeMatches(frame, map_previd_newid);

    return true;
}

void Mapper::mergeMatches(const Frame &frame, const std::map<int,int> &map_kpids_lmids)
{
    std::lock_guard<std::mutex> lock2(pmap_->optim_mutex_);

    std::lock_guard<std::mutex> lock(pmap_->map_mutex_);

    // Merge the matches
    for( const auto &ids : map_kpids_lmids )
    {
        int prevlmid = ids.first;
        int newlmid = ids.second;

        pmap_->mergeMapPoints(prevlmid, newlmid);
    }
}

std::map<int,int> Mapper::matchToMap(const Frame &frame, const float fmaxprojerr, const float fdistratio, std::unordered_set<int> &set_local_lmids)
{
    std::map<int,int> map_previd_newid;

    // Leave if local map is empty
    if( set_local_lmids.empty() ) {
        return map_previd_newid;
    }

    // Compute max field of view
    const float vfov = 0.5 * frame.pcalib_leftcam_->img_h_ / frame.pcalib_leftcam_->fy_;
    const float hfov = 0.5 * frame.pcalib_leftcam_->img_w_ / frame.pcalib_leftcam_->fx_;

    float maxradfov = 0.;
    if( hfov > vfov ) {
        maxradfov = std::atan(hfov);
    } else {
        maxradfov = std::atan(vfov);
    }

    const float view_th = std::cos(maxradfov);

    // Define max distance from projection
    float dmaxpxdist = fmaxprojerr;
    if( frame.nb3dkps_ < 30 ) {
        dmaxpxdist *= 2.;
    }

    std::map<int, std::vector<std::pair<int, float>>> map_kpids_vlmidsdist;

    // Go through all MP from the local map
    for( const int lmid : set_local_lmids )
    {
        if( bnewkfavailable_ ) {
            break;
        }

        if( frame.isObservingKp(lmid) ) {
            continue;
        }

        auto plm = pmap_->getMapPoint(lmid);

        if( plm == nullptr ) {
            continue;
        } else if( !plm->is3d_ || plm->desc_.empty() ) {
            continue;
        }

        Eigen::Vector3d wpt = plm->getPoint();

        //Project 3D MP into KF's image
        Eigen::Vector3d campt = frame.projWorldToCam(wpt);

        if( campt.z() < 0.1 ) {
            continue;
        }

        float view_angle = campt.z() / campt.norm();

        if( fabs(view_angle) < view_th ) {
            continue;
        }

        cv::Point2f projpx = frame.projCamToImageDist(campt);

        if( !frame.isInImage(projpx) ) {
            continue;
        }

        // Get all the kps around the MP's projection
        auto vnearkps = frame.getSurroundingKeypoints(projpx);

        // Find two best matches
        float mindist = plm->desc_.cols * fdistratio * 8.; // * 8 to get bits size
        int bestid = -1;
        int secid = -1;

        float bestdist = mindist;
        float secdist = mindist;

        std::vector<int> vkpids;
        std::vector<float> vpxdist;
        cv::Mat descs;

        for( const auto &kp : vnearkps )
        {
            if( kp.lmid_ < 0 ) {
                continue;
            }

            float pxdist = cv::norm(projpx - kp.px_);

            if( pxdist > dmaxpxdist ) {
                continue;
            }

            // Check that this kp and the MP are indeed
            // candidates for matching (by ensuring that they
            // are never both observed in a given KF)
            auto pkplm = pmap_->getMapPoint(kp.lmid_);

            if( pkplm == nullptr ) {
                pmap_->removeMapPointObs(kp.lmid_,frame.kfid_);
                continue;
            }

            if( pkplm->desc_.empty() ) {
                continue;
            }
            bool is_candidate = true;
            auto set_plmkfs = plm->getKfObsSet();
            for( const auto &kfid : pkplm->getKfObsSet() ) {
                if( set_plmkfs.count(kfid) ) {
                    is_candidate = false;
                    break;
                }
            }
            if( !is_candidate ) {
                continue;
            }

            float coprojpx = 0.;
            size_t nbcokp = 0;

            for( const auto &kfid : pkplm->getKfObsSet() ) {
                auto pcokf = pmap_->getKeyframe(kfid);
                if( pcokf != nullptr ) {
                    auto cokp = pcokf->getKeypointById(kp.lmid_);
                    if( cokp.lmid_ == kp.lmid_ ) {
                        coprojpx += cv::norm(cokp.px_ - pcokf->projWorldToImageDist(wpt));
                        nbcokp++;
                    } else {
                        pmap_->removeMapPointObs(kp.lmid_, kfid);
                    }
                } else {
                    pmap_->removeMapPointObs(kp.lmid_, kfid);
                }
            }

            if( coprojpx / nbcokp > dmaxpxdist ) {
                continue;
            }
            
            float dist = plm->computeMinDescDist(*pkplm);

            if( dist <= bestdist ) {
                secdist = bestdist; // Will stay at mindist 1st time
                secid = bestid; // Will stay at -1 1st time

                bestdist = dist;
                bestid = kp.lmid_;
            }
            else if( dist <= secdist ) {
                secdist = dist;
                secid = kp.lmid_;
            }
        }

        if( bestid != -1 && secid != -1 ) {
            if( 0.9 * secdist < bestdist ) {
                bestid = -1;
            }
        }

        if( bestid < 0 ) {
            continue;
        }

        std::pair<int, float> lmid_dist(lmid, bestdist);
        if( !map_kpids_vlmidsdist.count(bestid) ) {
            std::vector<std::pair<int, float>> v(1,lmid_dist);
            map_kpids_vlmidsdist.emplace(bestid, v);
        } else {
            map_kpids_vlmidsdist.at(bestid).push_back(lmid_dist);
        }
    }

    for( const auto &kpid_vlmidsdist : map_kpids_vlmidsdist )
    {
        int kpid = kpid_vlmidsdist.first;

        float bestdist = 1024;
        int bestlmid = -1;

        for( const auto &lmid_dist : kpid_vlmidsdist.second ) {
            if( lmid_dist.second <= bestdist ) {
                bestdist = lmid_dist.second;
                bestlmid = lmid_dist.first;
            }
        }

        if( bestlmid >= 0 ) {
            map_previd_newid.emplace(kpid, bestlmid);
        }
    }

    return map_previd_newid;
}


void Mapper::runFullBA()
{
    bool use_robust_cost = true;
    pestimator_->poptimizer_->fullBA(use_robust_cost);
}


bool Mapper::getNewKf(Keyframe &kf)
{
    std::lock_guard<std::mutex> lock(qkf_mutex_);

    // Check if new KF is available
    if( qkfs_.empty() ) {
        bnewkfavailable_ = false;
        return false;
    }

    // Get new KF and signal BA to stop if
    // it is still processing the previous KF
    kf = qkfs_.front();
    qkfs_.pop();

    // Setting bnewkfavailable_ to true will limit
    // the processing of the KF to triangulation and postpone
    // other costly tasks to next KF as we are running late!
    if( qkfs_.empty() ) {
        bnewkfavailable_ = false;
    } else {
        bnewkfavailable_ = true;
    }

    return true;
}


void Mapper::addNewKf(const Keyframe &kf)
{
    std::lock_guard<std::mutex> lock(qkf_mutex_);

    qkfs_.push(kf);

    bnewkfavailable_ = true;
}

void Mapper::reset()
{
    std::lock_guard<std::mutex> lock2(pmap_->optim_mutex_);

    bnewkfavailable_ = false;
    bwaiting_for_lc_ = false;
    bexit_required_ = false; 

    std::queue<Keyframe> empty;
    std::swap(qkfs_, empty);
}

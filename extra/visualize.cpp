#include "visualize.hpp"


Visualize::Visualize(std::shared_ptr<SlamManager> pslam)
  : pslam_(pslam)
{
}

void Visualize::run() {
  pangolin::CreateWindowAndBind("global mapping",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
              pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
              pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
              );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));

  Twc_.SetIdentity();

  cv::namedWindow("current frame");

  while( !bexit_required_ ) 
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updateCamrea();
    s_cam.Follow(Twc_);

    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    drawCamrea();
    drawGraph();

    drawKeyFrames();
    drawMapPoints();

    pangolin::FinishFrame();

    cv::Mat im = pslam_->visualFrame();

    if( !im.empty() ) { 
      cv::imshow("current frame", im);
      cv::waitKey(1000 / 30);
    }
    
  }
}

void Visualize::updateCamrea()
{
  Eigen::Matrix4d Twc = pslam_->pcurframe_->getTwc().matrix();
  vpos_.push_back(pslam_->pcurframe_->gettwc());
  
  Twc_.m[0] = Twc(0,0);
  Twc_.m[1] = Twc(1,0);
  Twc_.m[2] = Twc(2,0);
  Twc_.m[3]  = 0.0;

  Twc_.m[4] = Twc(0,1);
  Twc_.m[5] = Twc(1,1);
  Twc_.m[6] = Twc(2,1);
  Twc_.m[7]  = 0.0;

  Twc_.m[8] = Twc(0,2);
  Twc_.m[9] = Twc(1,2);
  Twc_.m[10] = Twc(2,2);
  Twc_.m[11]  = 0.0;

  Twc_.m[12] = Twc(0,3);
  Twc_.m[13] = Twc(1,3);
  Twc_.m[14] = Twc(2,3);
  Twc_.m[15]  = 1.0;
}

void Visualize::drawCamrea() {
  const float w = 0.08; // camera size;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();
  glMultMatrixd(Twc_.m);
  glLineWidth(3); // camera line width

  glColor3f(0.0f,1.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(w,h,z);
  glVertex3f(0,0,0);
  glVertex3f(w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,h,z);

  glVertex3f(w,h,z);
  glVertex3f(w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(-w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(w,h,z);

  glVertex3f(-w,-h,z);
  glVertex3f(w,-h,z);
  glEnd();

  glPopMatrix();

}

void Visualize::drawMapPoints() 
{

  // whether to add a lock ?
  size_t sz;
  {
    std::lock_guard<std::mutex> lock(pslam_->pmap_->lm_mutex_);
    sz = pslam_->pmap_->map_plms_.size();
  }
  
  if(sz == 0)
    return;

  glPointSize(4);
  glBegin(GL_POINTS);
  
  for(size_t i = 0; i < sz; i++) {
    auto plm = pslam_->pmap_->getMapPoint(i);
    
    if(plm == nullptr)
      continue;

    if(plm->isBad()) {
      glColor3f(0.0,0.0,0.0);
    } else {
      glColor3f(1.0,0.0,0.0);
    }

    Eigen::Vector3d& xyz = plm->ptxyz_;
    float x = xyz(0), y = xyz(1), z = xyz(2);
    glVertex3f(x,y,z);
  }

  glEnd();

}

void Visualize::drawKeyFrames() {
  const float w = 0.05;
  const float h = w*0.75;
  const float z = w*0.6;

  size_t sz;
  
  {
    std::lock_guard<std::mutex> lock(pslam_->pmap_->kf_mutex_);
    sz = pslam_->pmap_->map_pkfs_.size();
  }
  

  if(sz == 0)
    return;
  
  for(size_t i = 0; i < sz; i++) {
    auto pkf = pslam_->pmap_->getKeyframe(i);
    if(pkf == nullptr)
      continue;

    auto Twc = pkf->getTwc().matrix();
    cv::Mat twc;
    cv::eigen2cv(Twc, twc);
    twc = twc.t();

    glPushMatrix();

    glMultMatrixd(twc.ptr<GLdouble>(0));

    glLineWidth(1);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
  }
}

void Visualize::drawGraph() {
  glLineWidth(0.9); // graph line width
  glColor4f(0.0f,1.0f,0.0f,0.6f);
  glBegin(GL_LINES);

  for(int i = 1; i < vpos_.size(); i++) {
    glVertex3d(vpos_[i-1](0), vpos_[i-1](1), vpos_[i-1](2));
    glVertex3d(vpos_[i](0), vpos_[i](1), vpos_[i](2));
  }

  glEnd();
}


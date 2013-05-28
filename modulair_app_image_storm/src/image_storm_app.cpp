/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Johns Hopkins University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com
 */

#include <modulair_app_image_storm/image_storm_app.h>

namespace modulair{

  ImageStormApp::ImageStormApp(QString app_name, ros::NodeHandle nh, int event_deque_size) : ModulairAppBase(app_name, nh, event_deque_size){

    // readConfigFile();
    // setImageDirectories();

    // this->suspended = false;
    // this->useKinect = useKin;
    // this->runtime = 0;
    this->paused = false;

    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(updateApp()));
    // connect(this,SIGNAL(destroyed()),myManager,SLOT(confirmDestroyed()));
  }

  ImageStormApp::~ImageStormApp(){
    root = NULL;
    for(unsigned int i = 0; i<_envWrapper->getNumChildren();i++){
      _envWrapper->removeChild(i);
    }
    _envWrapper = NULL;

    for(unsigned int i = 0; i<usage_wrapper_->getNumChildren();i++){
      usage_wrapper_->removeChild(i);
    }
    usage_wrapper_ = NULL;

    for(unsigned int i = 0; i<plane_wrapper_->getNumChildren();i++){
      plane_wrapper_->removeChild(i);
    }
    plane_wrapper_ = NULL;

    cerr<<"DELETING PLANES"<<endl;
    for (int i=0; i< back_planes_.size(); i++){
      back_planes_.takeAt(0) = NULL;
    }
    this->back_planes_.clear();
    cerr<<"DELETING PLANES"<<endl;
    for (int i=0; i< usage_planes_.size(); i++){
      usage_planes_.takeAt(0) = NULL;
    }
    this->usage_planes_.clear();

    cerr<<"DELETING TEXTURES"<<endl;
    for(int i=0;i<this->_assetTextures.length();i++)
    {
      this->_assetTextures.takeAt(0) = NULL;
    }
    this->_assetTextures.clear();
  }

  void ImageStormApp::LoadTextures(){
    // Asset Textures //
    QDir asset_dir(this->asset_path_);
    QStringList assetFiles = asset_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<assetFiles.count(); i++){
      this->assetPaths << asset_dir.absoluteFilePath(assetFiles[i]);
    }

    ROS_WARN_STREAM("ImageStormApp: Loading Images");
    osg::ref_ptr<osg::Image> img;
    for (int i = 0; i < this->assetPaths.size(); i++){
      img = osgDB::readImageFile(this->assetPaths.at(i).toStdString());
      osg::ref_ptr<osg::TextureRectangle> texref =
          new osg::TextureRectangle(img);
      this->_assetTextures.push_back(texref);
    }
    ROS_WARN_STREAM(" Done");
  }

  bool ImageStormApp::build(){
    // Variables
    this->num_planes=0;
    this->sz = 150;
    this->camera_position_offset = 5000;
    
    std::string asset_path;
    if (!node_.getParam("/modulair/apps/image_storm_app/paths/assets", asset_path)){
      ROS_ERROR("Modulair%s: No asset path found on parameter server (namespace: %s)",
        name_.toStdString().c_str(), node_.getNamespace().c_str());
      return false;
    }else{
      asset_path_ = QString(asset_path.c_str());
      ROS_WARN_STREAM("ImageStormApp:  Asset path is [" << this->asset_path_.toStdString() << "]");
    }


    // Configure ImageStormApp
    // this->resize(width_,height_);
    // this->move(0,0);
    // this->show();

    root = new osg::Group;
    LoadTextures();

    // OSG Object Wrappers
    plane_wrapper_ = new OSGObjectBase();
    usage_wrapper_ = new OSGObjectBase();

    // good old shuffle
    int img_fairness[800];
    for (int i = 0; i < 800; i++)
    {
      img_fairness[i]=i%(this->assetPaths.size()-1);
    }
    for(int i=0;i<100;i++)
    {
      for(int j=0;j<800;j++)
      {
        int r=j+(rand()%(800-j));
        int tmp=img_fairness[j];
        img_fairness[j]=img_fairness[r];
        img_fairness[r]=tmp;
      }
    }

    for(int i = -20;i<20;i++){
      for (int j = -10; j< 10; j++)
      {
        // int imgnum;
        plane_start_pos.push_back(osg::Vec3(i*2*sz,j*2*sz,0));
        // printv(plane_start_pos[num_planes]);
        osg::ref_ptr<PlanarObject> s = new PlanarObject( -sz+5, -sz+5, 0,
                                                         sz-5, sz-5, 0, &_assetTextures,
                                                         img_fairness[num_planes],
                                                         plane_start_pos[num_planes]);
        plane_wrapper_->addChild(s.get());
        back_planes_.push_back(s.get());
        num_planes++;
        image_assignments.push_back(-1);
      }
    }
    ROS_WARN_STREAM("# of planes: "<<num_planes);
    for(int j=0;j<12;j++)
    {
      activeUsers[j]=false;
      prev_activeUsers[j]=false;
      joint_increments[j]=0;

    }

    for (int i = 0; i < 12; i++)
    {

      osg::ref_ptr<PlanarObject> s = new PlanarObject( -sz+5, -sz+5, 0,
                                                       sz-5, sz-5, 0, &_assetTextures,
                                                       this->assetPaths.size()-1,
                                                       osg::Vec3(0,-10000,0));
      usage_wrapper_->addChild(s.get());
      usage_planes_.push_back(s.get());
    }

    // TRANSPARENCY ///////////////////////////////////////////////////////////
    osg::StateSet* ss = root->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    // ENVIRONMENT ////////////////////////////////////////////////////////////
    ROS_WARN_STREAM("<<< ImageStorm >>> Setting Up Environment... ");
    _envWrapper = new OSGObjectBase();
    _envWrapper->addChild(plane_wrapper_);
    _envWrapper->addChild(usage_wrapper_);

    root->addChild(_envWrapper);
    ROS_WARN_STREAM("Done.");

    // GL QT WIDGET ///////////////////////////////////////////////////////////
    glWidget = new osgQt::GLWidget(this);
    glWidget = addViewWidget( createCamera(0,0,60,40,
                                           glWidget, "mainCamera", false), root);
    QGridLayout* grid = new QGridLayout;
    grid->addWidget( glWidget);
    setLayout( grid );
    ROS_WARN_STREAM("<<< ImageStorm >>> Created Widget");

    // STARTUP ////////////////////////////////////////////////////////////////
    _timer.start( 10 );
    _dataTimer.start(30);
    ROS_WARN_STREAM("<<< ImageStorm >>> Timers Started");
    // suspended = false;
    ROS_WARN_STREAM("<<< ImageStorm >>> Configured Successfully");

    // ROS_WARN_STREAM("Showing Menu Tip");
    // showMenuTip();
    return true;
  }

  bool ImageStormApp::start(){
    return true;
  }

  bool ImageStormApp::stop(){
    return true;
  }

  bool ImageStormApp::pause(){
    this->hide();
    _timer.stop();
    _dataTimer.stop();
    paused = true;
    ROS_WARN_STREAM("<< ImageStorm >> Pausing");

    return true;
  }

  bool ImageStormApp::resume(){
    _timer.start(10);
    _dataTimer.start(30);
    this->show();
    this->glWidget->show();
    this->update();
    paused = false;
    // this->suspended = false;
    ROS_WARN_STREAM("Showing Menu Tip");
    // showMenuTip();
    ROS_WARN_STREAM("<< ImageStorm >> Resumed");

    return true;
  }

  void ImageStormApp::updateApp(){
      // updateEnvironment();
  }

  /*void ImageStormApp::updateUsers(){
    numActiveUsers=0;
    for(int j=0;j<12;j++){
      activeUsers[j]=false;
    }
    UserPtrMap::iterator it;
    for(it = users.begin();it!=users.end();it++){
      int id = it->first;
      numActiveUsers++;
      LairUser* user = it->second;
      activeUsers[id]=true;
      //   ROS_WARN_STREAM( activeUsers[id] <<"aftersettrue");
      if(prev_activeUsers[id]==false)
      {
        //   ROS_WARN_STREAM("New user found! Selecting images for new user #"<<id);
        int assigned=0;
        while(assigned<IMAGES_PER_USER)
        {

          int toget = rand()%num_planes;
          //   ROS_WARN_STREAM("Chose image "<<toget);
          if(image_assignments[toget]==-1)
          {
            //   ROS_WARN_STREAM("It was valid!");
            image_assignments[toget]=id;
            assigned++;
          }
          else
          {
            //   ROS_WARN_STREAM("It was already being used by user "<<image_assignments[toget]<<" :(");
          }
        }
      }
      prev_activeUsers[id]=true;

    }

    for(int j=0;j<12;j++)
    {
      //   ROS_WARN_STREAM( "user j is "<<activeUsers[j] );
      if((activeUsers[j]==false)&&prev_activeUsers[j])
      {
        //   ROS_WARN_STREAM( activeUsers[j] <<"checkinguserj "<<j);
        prev_activeUsers[j]=false;
        for(int i=0;i<num_planes;i++)
        {

          if(image_assignments[i]==j)
          {
            //   ROS_WARN_STREAM("Image "<<i<<" was de-assigned");
            image_assignments[i]=-1;
          }
        }
      }
    }
  }*/

  /*void ImageStormApp::updateEnvironment(){
    updateUsers();

    // pull images to users
    for(int j=0;j<12;j++)
    {
      joint_increments[j]=0;
      if(!activeUsers[j]){
        usage_planes_[j]->setPos3DAbs( osg::Vec3(0,-10000,0));
      }
    }
    for(int i=0;i<num_planes;i++){
      if(image_assignments[i]==-1){
        osg::Vec3 curp=back_planes_[i]->getPos3DVec3();
        osg::Vec3 psp = plane_start_pos[i];
        back_planes_[i]->setPos3DRel( (psp-curp)*0.1 );
      }else{
        int ii=image_assignments[i];
        int jj=joint_increments[ii];
        //   ROS_WARN_STREAM(jj);
        vct3 joint_vec=users[ image_assignments[i] ]->pts3D[jj];

        joint_vec[0]*=1.3;
        joint_vec[1]-=500;

        // CAP JOINT Z to keep images small
        if(joint_vec[2] > 2500)
          joint_vec[2] = 2500;

        joint_increments[image_assignments[i]]++;

        if(jj==TOR){
          joint_vec[2]+=70;
          usage_planes_[image_assignments[i]]->setPos3DRel( (joint_vec-usage_planes_[image_assignments[i]]->getPos3D())*0.5 );
        }
        else{

          vct3 torso=users[ image_assignments[i] ]->pts3D[TOR];
          if(joint_vec[2]>=torso[2])
          {
            joint_vec[2]=torso[2]-50;
          }

          back_planes_[i]->setPos3DRel( (joint_vec-back_planes_[i]->getPos3D())*0.1 );
        }
      }
    }
    collide();
  }*/

  /*//////////////////////////////////////////////////////////////////////////*/

  /* OSG Viewer and Camera*/
  osgQt::GLWidget* ImageStormApp::addViewWidget( osg::Camera* camera, osg::Node* scene ){
    setCamera( camera );

    setSceneData( scene );
    osg::ref_ptr<osgViewer::StatsHandler> stats =
        new osgViewer::StatsHandler;
    addEventHandler(stats);
    //setCameraManipulator( new osgGA::TrackballManipulator );

    KeyboardHandler* myFirstEventHandler =
        new KeyboardHandler();
    myFirstEventHandler->setup(this);
    addEventHandler(myFirstEventHandler);
    setThreadingModel(SingleThreaded);
    osgQt::GraphicsWindowQt* gw =
        dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );

    return gw ? gw->getGLWidget() : NULL;
  }

  osg::Camera* ImageStormApp::createCamera( int x, int y, int w, int h, osgQt::GLWidget *QTObject, const  string& name, bool windowDecoration){
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = windowDecoration;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    osg::ref_ptr<osgQt::GraphicsWindowQt::WindowData> inheritWindow=
        new osgQt::GraphicsWindowQt::WindowData(NULL, QTObject);
    traits->inheritedWindowData = inheritWindow.get();
    _camera =
        new osg::Camera;
    osg::ref_ptr<osgQt::GraphicsWindowQt> gwqt =
        new osgQt::GraphicsWindowQt(traits.get());
    _camera->setGraphicsContext(gwqt.get());

    _camera->setViewMatrixAsLookAt( osg::Vec3d( 0,0,camera_position_offset ), // eye
                                    osg::Vec3d( 0,0,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    osg::ref_ptr<osg::Viewport> view = new osg::Viewport(0,0,w,h);
    _camera->setViewport(view);
    _camera->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(w)/static_cast<double>(h), 0.1f, 100000.0f );

    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,50,0);

    return _camera;
  }

  /* Keyboard Handler */
  bool KeyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa){
    switch(ea.getEventType())
    {
    case(osgGA::GUIEventAdapter::KEYDOWN):
    {
      switch(ea.getKey())
      {
      case 'q':
        break;

        // Camera Zoom
      case 'n':
        break;
      case 'm':
        break;
      case 'x':
        break;
      case 'c':
        // this->appPtr->collide();
        break;
      case 'v':
        break;
      case 'b':
        break;
      case 'f':
        break;
      case 'h':
        break;
      }
    }
    default:
      return false;
    }
  }

  void KeyboardHandler::accept(osgGA::GUIEventHandlerVisitor& v){ 
    v.visit(*this);
  }

  void KeyboardHandler::setup(ImageStormApp* appPt){
    this->appPtr = appPt;
  }

} // end namespace modulair

/*
 * Main File
 */
using namespace modulair;

int main(int argc, char* argv[]){
  ros::init(argc,argv, "image_storm_app");
  ROS_WARN_STREAM("ImageStormApp: Starting Up...");
  ros::NodeHandle node_handle;
  QApplication application(argc,argv);
  // This line will quit the application once any window is closed.
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
  modulair::ImageStormApp image_storm_app("ImageStormApp",node_handle,20);
  image_storm_app.build();
  ROS_WARN_STREAM("ImageStormApp: App Running");
  application.exec();
  // Running
  image_storm_app.stop();
  ROS_WARN_STREAM("ImageStormApp: App Finished");
  return 0;
}
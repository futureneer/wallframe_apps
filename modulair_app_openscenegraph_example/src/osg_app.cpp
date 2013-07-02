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

#include <modulair_app_openscenegraph_example/osg_app.h>
#include <modulair_osg_tools/vector_conversions.h>

namespace modulair{

  OsgApp::OsgApp(std::string app_name, ros::NodeHandle nh, int event_deque_size)
    : ModulairAppBaseQt(app_name, nh, event_deque_size){
    this->paused = false;
    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(updateApp()));
  }

  OsgApp::~OsgApp(){
    root = NULL;
    for(unsigned int i = 0; i<_envWrapper->getNumChildren();i++){
      _envWrapper->removeChild(i);
    }
    _envWrapper = NULL;
    // usage_wrapper_ = NULL;
    for(unsigned int i = 0; i<plane_wrapper_->getNumChildren();i++){
      plane_wrapper_->removeChild(i);
    }
    plane_wrapper_ = NULL;
    cerr<<"DELETING PLANES"<<endl;
    for (int i=0; i< back_planes_.size(); i++){
      back_planes_.takeAt(0) = NULL;
    }
    this->back_planes_.clear();
    cerr<<"DELETING TEXTURES"<<endl;
    for(int i=0;i<this->_assetTextures.length();i++)
    {
      this->_assetTextures.takeAt(0) = NULL;
    }
    this->_assetTextures.clear();
  }

  void OsgApp::LoadTextures(){
    // Asset Textures //
    QDir asset_dir(this->asset_path_);
    QStringList assetFiles = asset_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<assetFiles.count(); i++){
      this->assetPaths << asset_dir.absoluteFilePath(assetFiles[i]);
    }
    ROS_WARN_STREAM("OsgApp: Loading Images");
    osg::ref_ptr<osg::Image> img;
    for (int i = 0; i < this->assetPaths.size(); i++){
      img = osgDB::readImageFile(this->assetPaths.at(i).toStdString());
      osg::ref_ptr<osg::TextureRectangle> texref =
          new osg::TextureRectangle(img);
      this->_assetTextures.push_back(texref);
    }
    ROS_WARN_STREAM(" Done");
  }

  bool OsgApp::build(){
    // VARIABLES ///////////////////////////////////////////////////////////////
    this->num_planes = 0;
    this->plane_size_ = 150;
    this->camera_position_offset = 5000;
    // IMAGE ASSETS ////////////////////////////////////////////////////////////
    std::string asset_path;
    if (!node_.getParam("/modulair/apps/osg_app/paths/assets", asset_path)){
      ROS_ERROR("Modulair%s: No asset path found on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }else{
      asset_path_ = QString(asset_path.c_str());
      ROS_WARN_STREAM("OsgApp:  Asset path is [" << this->asset_path_.toStdString() << "]");
    }
    // OSG OBJECTS /////////////////////////////////////////////////////////////
    root = new osg::Group;
    LoadTextures();
    // OSG Object Wrappers
    plane_wrapper_ = new OSGObjectBase();
    // IMAGE PLANES ////////////////////////////////////////////////////////////
    plane_start_pos.push_back(osg::Vec3(.5*plane_size_,.5*plane_size_,0));
    osg::ref_ptr<PlanarObject> s = new PlanarObject( -plane_size_+5, -plane_size_+5, 0,
                                                     plane_size_-5, plane_size_-5, 0, 
                                                     &_assetTextures,
                                                     num_planes,
                                                     plane_start_pos[num_planes]);
    plane_wrapper_->addChild(s.get());
    back_planes_.push_back(s.get());
    num_planes++;
    // TRANSPARENCY ////////////////////////////////////////////////////////////
    osg::StateSet* ss = root->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    // ENVIRONMENT /////////////////////////////////////////////////////////////
    ROS_WARN_STREAM("<<< ImageStorm >>> Setting Up Environment... ");
    _envWrapper = new OSGObjectBase();
    _envWrapper->addChild(plane_wrapper_);
    // _envWrapper->addChild(usage_wrapper_);
    root->addChild(_envWrapper);
    ROS_WARN_STREAM("Done.");
    // GL QT WIDGET ////////////////////////////////////////////////////////////
    glWidget = new osgQt::GLWidget(this);
    glWidget = addViewWidget( createCamera(0,0,60,40,
                                           glWidget, "mainCamera", false), root);
    QGridLayout* grid = new QGridLayout;
    grid->addWidget( glWidget);
    setLayout( grid );
    ROS_WARN_STREAM("<<< ImageStorm >>> Created Widget");
    // DONE //
    ROS_WARN_STREAM("<<< ImageStorm >>> Configured Successfully");
    return true;
  }

  bool OsgApp::start(){
    // Start up app
    _timer.start( 10 );
    _dataTimer.start(10);
    ROS_WARN_STREAM("<<< ImageStorm >>> Timers Started");
    return true;
  }

  bool OsgApp::stop(){
    /*Stop stuff here before destructor is called, if needed*/
    return true;
  }

  bool OsgApp::pause(){
    this->hide();
    _timer.stop();
    _dataTimer.stop();
    paused = true;
    ROS_WARN_STREAM("<< ImageStorm >> Pausing");
    return true;
  }

  bool OsgApp::resume(){
    _timer.start(10);
    _dataTimer.start(10);
    this->show();
    this->glWidget->show();
    this->update();
    paused = false;
    ROS_WARN_STREAM("<< ImageStorm >> Resumed");
    return true;
  }

  void OsgApp::updateApp(){
    updateEnvironment();
  }

  void OsgApp::updateEnvironment(){
    if(num_users_ == 0){
      back_planes_[0]->setPos3DRel( (plane_start_pos[0]-back_planes_[0]->getPos3D())*0.1 );
    }else{
      AppUser user;
      getFocusedUser(user);
      osg::Vec3 pos = eigToOsg3(user.jtPosByName("torso"));
      pos[0]*=1.3;
      pos[1]-=200;
      if(pos[2] > 2500)
        pos[2] = 2500;
      back_planes_[0]->setPos3DRel( (pos-back_planes_[0]->getPos3D())*0.1 );
    }
  }

  /*//////////////////////////////////////////////////////////////////////////*/

  /* OSG Viewer and Camera*/
  osgQt::GLWidget* OsgApp::addViewWidget( osg::Camera* camera, osg::Node* scene ){
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

  osg::Camera* OsgApp::createCamera( int x, int y, int w, int h, osgQt::GLWidget *QTObject, const  string& name, bool windowDecoration){
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

  void KeyboardHandler::setup(OsgApp* appPt){
    this->appPtr = appPt;
  }

} // end namespace modulair

/*
 * Main File
 */
using namespace modulair;

int main(int argc, char* argv[]){
  ros::init(argc,argv, "osg_app");
  ROS_WARN_STREAM("OsgApp: Starting Up...");
  ros::NodeHandle node_handle;
  QApplication application(argc,argv);
  // This line will quit the application once any window is closed.
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
  modulair::OsgApp osg_app("OsgApp",node_handle,20);
  osg_app.build();
  osg_app.start();
  ROS_WARN_STREAM("OsgApp: App Running");
  application.exec();
  // Running
  osg_app.stop();
  ROS_WARN_STREAM("OsgApp: App Finished");
  return 0;
}


#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>

#include "wallcad.h"


namespace modulair {

WallCadApp::WallCadApp(std::string app_name, ros::NodeHandle nh, int event_deque_size)
  :ModulairAppBase(app_name, nh, event_deque_size)
{
  this->paused_ = false;
  connect(&timer_, SIGNAL(timeout()), this, SLOT(update()));
  connect(&data_timer_, SIGNAL(timeout()), this, SLOT(updateApp()));
}

WallCadApp::~WallCadApp()
{
  // clean up WallCadApp osg part
}


// ----- modulair appbase virtual methods -------
bool WallCadApp::build()
{
  // build stuff here

  // --- get image asset path ---
  std::string asset_path;
  if (!node_.getParam("/modulair/apps/wallcad_app/paths/assets", asset_path)){
    ROS_ERROR("Modulair%s: No asset path found on parameter server (namespace: %s)",
              name_.c_str(), node_.getNamespace().c_str());
    return false;
  }else{
    asset_path_ = QString(asset_path.c_str());
    ROS_WARN_STREAM("WallCadApp:  Asset path is [" << this->asset_path_.toStdString() << "]");
  }

  // ----- create OSG objects ------
  root_ = new osg::Group;

//  osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), 1.0);
//  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
//  geode->addDrawable(sphere);

  QWidget* widget1 = addViewWidget( createCamera(0,0,60, 40),
                                    osgDB::readNodeFile("cow.osg") );

//  root_->addChild(geode);

  ROS_WARN_STREAM("DONE");

  glwidget_ = new osgQt::GLWidget(this);
//  QGridLayout* gridLayout = new QGridLayout;
//  gridLayout->addWidget(widget1);
//  this->setLayout(gridLayout);

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(widget1);
  setLayout(layout);

  return true;
}

bool WallCadApp::start()
{
  // start up app
  timer_.start(50);
  data_timer_.start(50);
  this->show();
  ROS_WARN_STREAM("<<< WallCad >>> Timers Started");
  return true;
}

bool WallCadApp::stop()
{
  // stop stuff here before destructor is called
  return true;
}

bool WallCadApp::pause()
{
  this->hide();
  timer_.stop();
  data_timer_.stop();
  paused_ = true;
  ROS_WARN_STREAM("<<< WallCad >>> Paused");

  return true;
}

bool WallCadApp::resume()
{
  timer_.start(50);
  data_timer_.start(50);
  paused_ = false;
  this->show();
  this->update();
  ROS_WARN_STREAM("<<< WallCad >>> Resumed");

  return true;
}


// ------ osg qt ------------
QWidget* WallCadApp::addViewWidget(osg::Camera *camera, osg::Node *scene)
{
  setCamera(camera);
  setSceneData(scene);
  addEventHandler( new osgViewer::StatsHandler );
  setCameraManipulator( new osgGA::TrackballManipulator );

  osgQt::GraphicsWindowQt* gw =
      dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );

  // return pointer
  return gw ? gw->getGLWidget() : NULL;
}


osg::Camera* WallCadApp::createCamera(int x, int y, int w, int h,
                                      const std::string &name,
                                      bool windowDecoration)
{
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

  // NOTE: different from kel's version
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

  camera->setClearColor( osg::Vec4(0.2, 0.2, 0.6, 1.0) );
  camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
  camera->setProjectionMatrixAsPerspective(
      30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
  return camera.release();
}


// ----- modulair slots ------
void WallCadApp::update()
{
//  std::cerr << "update " << std::endl;
}

void WallCadApp::updateApp()
{
//  std::cerr << "updateApp " << std::endl;
}


}

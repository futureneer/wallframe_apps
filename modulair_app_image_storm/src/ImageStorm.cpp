#include "ImageStorm.h"
#include "OSGObjectBase.h"
#include <iostream>

#include <ros/ros.h>

using namespace osg;
using namespace lair;
using namespace std;

#define campos 5000
#define planepos 0
#define sz 150
#define IMAGES_PER_USER 13

int num_planes;

// ImageStorm::ImageStorm( QWidget *par, QWidget *appManager, QString appID, bool useKin) : AppBase(par, appID)
// {
//   this->myParent = par;
//   this->myID = appID;
//   this->myManager = appManager;

//   readConfigFile();
//   setImageDirectories();

//   this->suspended = false;
//   this->useKinect = useKin;
//   runtime = 0;
//   paused = false;

//   connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
//   connect( &_timer, SIGNAL(timeout()), this, SLOT(increment()));
//   connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(pullData()));
//   connect(this,SIGNAL(destroyed()),myManager,SLOT(confirmDestroyed()));
// }

// ImageStorm::~ImageStorm()
// {
//   root = NULL;
//   for(int i = 0; i<_envWrapper->getNumChildren();i++){
//     _envWrapper->removeChild(i);
//   }
//   _envWrapper = NULL;

//   for(int i = 0; i<usage_wrapper_->getNumChildren();i++){
//     usage_wrapper_->removeChild(i);
//   }
//   usage_wrapper_ = NULL;

//   for(int i = 0; i<plane_wrapper_->getNumChildren();i++){
//     plane_wrapper_->removeChild(i);
//   }
//   plane_wrapper_ = NULL;



//   cerr<<"DELETING PLANES"<<endl;
//   for (int i=0; i< back_planes_.size(); i++){
//     back_planes_.takeAt(0) = NULL;
//   }
//   this->back_planes_.clear();
//   cerr<<"DELETING PLANES"<<endl;
//   for (int i=0; i< usage_planes_.size(); i++){
//     usage_planes_.takeAt(0) = NULL;
//   }
//   this->usage_planes_.clear();

//   cerr<<"DELETING TEXTURES"<<endl;
//   for(int i=0;i<this->_assetTextures.length();i++)
//   {
//     this->_assetTextures.takeAt(0) = NULL;
//   }
//   this->_assetTextures.clear();
// }

// void ImageStorm::increment(){ runtime++;}

// void ImageStorm::config()
// {
//   double _pi = 3.14159265;
//   this->resize(W_WIDTH,W_HEIGHT-INFOBAR_HEIGHT);
//   this->move(0,0);
//   this->show();
//   root = new osg::Group;
//   if(useKinect)
//     ROS_WARN_STREAM("<<< ImageStorm >>> Application using kinect input.");

//   LoadTextures();

//   // Cursors
//   ROS_WARN_STREAM("Adding Cursors... ");
//   ROS_WARN_STREAM("Done.");

//   // Spheres
//   plane_wrapper_ = new OSGObjectBase();
//   usage_wrapper_ = new OSGObjectBase();


//   num_planes=0;


//   // good old shuffle
//   int img_fairness[800];
//   for (int i = 0; i < 800; i++)
//   {
//     img_fairness[i]=i%(this->assetPaths.size()-1);
//   }
//   for(int i=0;i<100;i++)
//   {
//     for(int j=0;j<800;j++)
//     {
//       int r=j+(rand()%(800-j));
//       int tmp=img_fairness[j];
//       img_fairness[j]=img_fairness[r];
//       img_fairness[r]=tmp;
//     }
//   }

//   for(int i = -20;i<20;i++){
//     for (int j = -10; j< 10; j++)
//     {
//       // int imgnum;
//       plane_start_pos.push_back(osg::Vec3(i*2*sz,j*2*sz,0));
//       // printv(plane_start_pos[num_planes]);
//       osg::ref_ptr<PlanarObject> s = new PlanarObject( -sz+5, -sz+5, 0,
//                                                        sz-5, sz-5, 0, &_assetTextures,
//                                                        img_fairness[num_planes],
//                                                        plane_start_pos[num_planes]);
//       plane_wrapper_->addChild(s.get());
//       back_planes_.push_back(s.get());
//       num_planes++;
//       image_assignments.push_back(-1);
//     }
//   }
//   ROS_WARN_STREAM("# of planes: "<<num_planes);
//   for(int j=0;j<12;j++)
//   {
//     activeUsers[j]=false;
//     prev_activeUsers[j]=false;
//     joint_increments[j]=0;

//   }

//   for (int i = 0; i < 12; i++)
//   {

//     osg::ref_ptr<PlanarObject> s = new PlanarObject( -sz+5, -sz+5, 0,
//                                                      sz-5, sz-5, 0, &_assetTextures,
//                                                      this->assetPaths.size()-1,
//                                                      osg::Vec3(0,-10000,0));
//     usage_wrapper_->addChild(s.get());
//     usage_planes_.push_back(s.get());
//   }

//   // TRANSPARENCY ///////////////////////////////////////////////////////////
//   osg::StateSet* ss = root->getOrCreateStateSet();
//   ss->setMode(GL_BLEND, osg::StateAttribute::ON);
//   ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
//   ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

//   // ENVIRONMENT ////////////////////////////////////////////////////////////
//   ROS_WARN_STREAM("<<< ImageStorm >>> Setting Up Environment... ");
//   _envWrapper = new OSGObjectBase();
//   _envWrapper->addChild(plane_wrapper_);
//   _envWrapper->addChild(usage_wrapper_);

//   root->addChild(_envWrapper);
//   ROS_WARN_STREAM("Done.");

//   // GL QT WIDGET ///////////////////////////////////////////////////////////
//   glWidget = new osgQt::GLWidget(this);
//   glWidget = addViewWidget( createCamera(0,0,60,40,
//                                          glWidget, "mainCamera", false), root);
//   QGridLayout* grid = new QGridLayout;
//   grid->addWidget( glWidget);
//   setLayout( grid );
//   ROS_WARN_STREAM("<<< ImageStorm >>> Created Widget");

//   // STARTUP ////////////////////////////////////////////////////////////////
//   _timer.start( 10 );
//   _dataTimer.start(30);
//   ROS_WARN_STREAM("<<< ImageStorm >>> Timers Started");
//   suspended = false;o
//   ROS_WARN_STREAM("<<< ImageStorm >>> Configured Successfully");


//   ROS_WARN_STREAM("Showing Menu Tip");
//   showMenuTip();
// }

void ImageStorm::pullData()
{
  if(this->useKinect){
    requestUserData();
    updateEnvironment();
  }
}

void ImageStorm::updateUsers()
{
  //   ROS_WARN_STREAM("1=====num_planes: "<<num_planes);
  numActiveUsers=0;
  for(int j=0;j<12;j++)
  {
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

}

void ImageStorm::collide()
{
  for (int i = 0; i < back_planes_.size(); i++)
  {
    if(image_assignments[i]!=-1)
    {
      const osg::BoundingSphereImpl<osg::Vec3f> s1 = back_planes_[i]->getBound();
      for (int j = i+1; j < back_planes_.size(); j++)
      {
        if(image_assignments[j]!=-1)
        {
          const osg::BoundingSphereImpl<osg::Vec3f> s2 = back_planes_[j]->getBound();

          if(s1.intersects(s2)){
            osg::Vec3 dir = s1.center()-s2.center();
            double center_dist = dir.length();
            double dist = -(center_dist-sz)*0.1;
            dir.normalize();
            // dir[2]=2;
            osg::Vec3 mv = dir*dist;
            back_planes_[j]->setPosition(back_planes_[j]->getPosition()+mv*0.2);
          }
        }
      }
    }
  }
}

void ImageStorm::updateEnvironment()
{
  updateUsers();

  // pull images to users
  for(int j=0;j<12;j++)
  {

    joint_increments[j]=0;

    if(!activeUsers[j])
    {
      usage_planes_[j]->setPos3DAbs( osg::Vec3(0,-10000,0));
    }

  }

  for(int i=0;i<num_planes;i++)
  {
    if(image_assignments[i]==-1)
    {

      osg::Vec3 curp=back_planes_[i]->getPos3DVec3();

      osg::Vec3 psp = plane_start_pos[i];

      back_planes_[i]->setPos3DRel( (psp-curp)*0.1 );
    }
    else
    {
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
}

// void ImageStorm::updateCursors()
// {
  
// }

void ImageStorm::pause()
{
  this->hide();
  _timer.stop();
  _dataTimer.stop();
  this->myParent->update();
  paused = true;
  ROS_WARN_STREAM("<< ImageStorm >> Pausing");
}

void ImageStorm::unpause()
{
  this->show();
  _timer.start(10);
  _dataTimer.start(30);
  this->update();
  paused = false;
  ROS_WARN_STREAM("Showing Menu Tip");
  showMenuTip();
  ROS_WARN_STREAM("<< ImageStorm >> Un-pausing");
}

void ImageStorm::resume()
{
  _timer.start(10);
  _dataTimer.start(30);
  this->show();
  this->glWidget->show();
  this->update();
  this->suspended = false;
  ROS_WARN_STREAM("Showing Menu Tip");
  showMenuTip();
  ROS_WARN_STREAM("<< ImageStorm >> Resumed");
}

void ImageStorm::suspend()
{
  _timer.stop();
  _dataTimer.stop();
  this->hide();
  this->glWidget->hide();
  this->myParent->update();
  this->suspended = true;
  ROS_WARN_STREAM("<< ImageStorm >> Suspended");
}

osgQt::GLWidget* ImageStorm::addViewWidget( osg::Camera* camera, osg::Node* scene )
{
  setCamera( camera );

  setSceneData( scene );
  osg::ref_ptr<osgViewer::StatsHandler> stats =
      new osgViewer::StatsHandler;
  addEventHandler(stats);
  //setCameraManipulator( new osgGA::TrackballManipulator );

  ImageStormKeyboardHandler* myFirstEventHandler =
      new ImageStormKeyboardHandler();
  myFirstEventHandler->setup(this);
  addEventHandler(myFirstEventHandler);
  setThreadingModel(SingleThreaded);
  osgQt::GraphicsWindowQt* gw =
      dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );

  return gw ? gw->getGLWidget() : NULL;
}

osg::Camera* ImageStorm::createCamera( int x, int y, int w, int h, osgQt::GLWidget *QTObject, const  string& name, bool windowDecoration)
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

  osg::ref_ptr<osgQt::GraphicsWindowQt::WindowData> inheritWindow=
      new osgQt::GraphicsWindowQt::WindowData(NULL, QTObject);
  traits->inheritedWindowData = inheritWindow.get();
  _camera =
      new osg::Camera;
  osg::ref_ptr<osgQt::GraphicsWindowQt> gwqt =
      new osgQt::GraphicsWindowQt(traits.get());
  _camera->setGraphicsContext(gwqt.get());

  _camera->setViewMatrixAsLookAt( osg::Vec3d( 0,0,campos ), // eye
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

// Keyboard Handler ////////////////////////////
bool ImageStormKeyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
  switch(ea.getEventType())
  {
  case(osgGA::GUIEventAdapter::KEYDOWN):
  {
    switch(ea.getKey())
    {
    case 'q':
      this->appPtr->sendSelfTerminate();
      break;

      // Camera Zoom
    case 'n':
      break;
    case 'm':
      break;
    case 'x':
      break;
    case 'c':
      this->appPtr->collide();
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

void ImageStormKeyboardHandler::accept(osgGA::GUIEventHandlerVisitor& v)
{ 
  v.visit(*this);
}

void ImageStormKeyboardHandler::setup(ImageStorm* appPt)
{
  this->appPtr = appPt;
}

// void ImageStorm::setImageDirectories()
// {
//   // Asset Textures //
//   QDir asset_dir(this->assetDir);
//   QStringList assetFiles = asset_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
//   for (int i=0; i<assetFiles.count(); i++){
//     this->assetPaths << asset_dir.absoluteFilePath(assetFiles[i]);
//   }
// }

// void ImageStorm::LoadTextures()
// {
//   ROS_WARN_STREAM("Loading Images ");
//   osg::ref_ptr<osg::Image> img;
//   for (int i = 0; i < this->assetPaths.size(); i++){
//     img = osgDB::readImageFile(this->assetPaths.at(i).toStdString());
//     osg::ref_ptr<osg::TextureRectangle> texref =
//         new osg::TextureRectangle(img);
//     this->_assetTextures.push_back(texref);
//   }
//   ROS_WARN_STREAM(" Done");
// }

// void ImageStorm::readConfigFile()
// {   
//   QString app_path(GetEnv("LAIR_APP_PATH").c_str());
//   ROS_WARN_STREAM("*** ImageStorm *** >> App Path: "<<app_path.toStdString());
  
//   QString configFileName = app_path + QString("/") + this->myID + QString("/")+ this->myID +QString(".txt");
//   QFile config_file(configFileName);
//   if (!config_file.open(QIODevice::ReadOnly)){
//     ROS_WARN_STREAM("*** ImageStorm Config File *** >> Error with config file");
//     ROS_WARN_STREAM("*** ImageStorm Config File *** >> File is "<<configFileName.toStdString());
//     exit(0);
//   }

//   ROS_WARN_STREAM("*** ImageStorm Config File *** >> Parsing Config File ... ");
//   QTextStream stream ( &config_file );
//   while( !stream.atEnd() ) {
//     QString line;
//     QStringList lineElem;
//     line = stream.readLine();
//     lineElem = line.split(" ");

//     if(lineElem[0] == "ASSET_DIR"){
//       this->assetDir = app_path + QString("/") + this->myID + QString("/")+lineElem[1];
//       ROS_WARN_STREAM(this->assetDir.toStdString());
//     }

//   }
// }

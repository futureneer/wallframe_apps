#include "WallBallWidget.h"

#include <Managers/GraphicsManager.h>

#include <iostream>

#include <QTimer>
#include <QDialog>
#include <QHBoxLayout>
#include <pthread.h>
using namespace std;
using namespace modulair;

bool WallBallWidget::s_RunPhysics = false;

pthread_t s_PhysicsThread;
void* WallBallWidget::physicsThreadMethod(void* data) {
    cout << "physicsThreadMethod()\n" << flush;
    while (s_RunPhysics) {
    	WallBall::tickPhysics();
    }
    return 0;
}

WallBallWidget::WallBallWidget(std::string app_name, ros::NodeHandle nh, int event_deque_size) :
    ModulairAppBaseQt(app_name, nh, event_deque_size)
{
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setFocusPolicy(Qt::StrongFocus);
    setAutoFillBackground(false);

    WallBall::start(this);
    
    QTimer* timer = new QTimer(this);
    connect( timer, SIGNAL( timeout() ), this, SLOT( update() ) );
    timer->start(1000.0 / 60.0);
    
    s_RunPhysics = true;
    pthread_create(&s_PhysicsThread, NULL, &(WallBallWidget::physicsThreadMethod), NULL);
    
    setFocus();
}

WallBallWidget::~WallBallWidget()
{
    s_RunPhysics = false;
}

void WallBallWidget::updateUsers(){
    numActiveUsers=0;
    for(int j=0;j<12;j++){
      activeUsers[j]=false;
    }
    
    AppUserMap::iterator uit;
    for(uit = users_.begin();uit!=users_.end();uit++){
      int id = uit->first;
      numActiveUsers++;
      AppUser user = uit->second;
      activeUsers[id]=true;
      /*
      //   ROS_WARN_STREAM( activeUsers[id] <<"aftersettrue");
      if(prev_activeUsers[id]==false){
        //   ROS_WARN_STREAM("New user found! Selecting images for new user #"<<id);
        int assigned=0;
        while(assigned < images_per_user_){
          int planes_to_get = rand()%num_planes;
          //   ROS_WARN_STREAM("Chose image "<<planes_to_get);
          if(image_assignments[planes_to_get]==-1){
            //   ROS_WARN_STREAM("It was valid!");
            image_assignments[planes_to_get]=id;
            assigned++;
          }else{
            //   ROS_WARN_STREAM("It was already being used by user "<<image_assignments[planes_to_get]<<" :(");
          }
        }
      }
      */
      prev_activeUsers[id]=true;
    }

    /*
    // Release Images from users who left
    for(int j=0;j<12;j++){
      //   ROS_WARN_STREAM( "user j is "<<activeUsers[j] );
      if((activeUsers[j]==false) && prev_activeUsers[j]){
        //   ROS_WARN_STREAM( activeUsers[j] <<"checkinguserj "<<j);
        prev_activeUsers[j]=false;
        for(int i=0;i<num_planes;i++){
          if(image_assignments[i]==j){
            //   ROS_WARN_STREAM("Image "<<i<<" was de-assigned");
            image_assignments[i]=-1;
          }
        }
      }
    }
    */
  }

bool WallBallWidget::build(){
    std::string asset_path;
    if (!node_.getParam("/modulair/apps/wallball_app/paths/assets", asset_path)){
      ROS_ERROR("Modulair%s: No asset path found on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    asset_path_ = QString(asset_path.c_str());
    return true;
  }

bool WallBallWidget::start(){return true;}
bool WallBallWidget::stop(){return true;}
bool WallBallWidget::pause(){return true;}
bool WallBallWidget::resume(){return true;}

/* Currently, app does not use ROS asset directory. Should add this method and modify it to do so.
void WallBallWidget::LoadTextures(){
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
*/

void WallBallWidget::doResize(int w, int h) {
    GraphicsManager::singleton()->resize(w, h);
}

void WallBallWidget::resizeEvent(QResizeEvent* event) {
    this->doResize(event->size().width(), event->size().height());
}

void WallBallWidget::paintEvent(QPaintEvent* event) {
    WallBall::tickGraphics();
}

void WallBallWidget::keyPressEvent(QKeyEvent *event) {
	int key = event->key();
	InputManager::setKeyDown(key, true);
}

void WallBallWidget::keyReleaseEvent(QKeyEvent *event) {
	int key = event->key();
	InputManager::setKeyDown(key, false);
}

int main(int argc, char* argv[]){
  // ros::init must be called before instantiating any apps
  ros::init(argc,argv, "wallball_app");
  ROS_WARN_STREAM("WallBallApp: Starting Up...");
  ros::NodeHandle node_handle;
  QApplication application(argc,argv);
  // This line will quit the application once any window is closed.
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
  WallBallWidget wallBallWidget("WallBallApp",node_handle,20);
  wallBallWidget.build();
  ROS_WARN_STREAM("WallBallApp: App Running");
  application.exec();
  // Running
  wallBallWidget.stop();
  ROS_WARN_STREAM("WallBallApp: App Finished");
  return 0;
}

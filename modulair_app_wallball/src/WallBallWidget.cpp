#include "WallBallWidget.h"

#include <Managers/GraphicsManager.h>
#include <Managers/InputManager.h>

#include <iostream>
#include <stdlib.h>

#include <QTimer>
#include <QDialog>
#include <QHBoxLayout>
#include <pthread.h>

using namespace std;
using namespace modulair;

#define NUM_USERS 7

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
    std::cout << "WallBallWidget constructor!!!\n" << std::flush;

    // INIT ASSET PATH
    if (!node_.getParam("/modulair/apps/wallball_app/paths/assets", WallBall::s_AssetPath)){
      ROS_ERROR("Modulair App WallBall: No asset path found on parameter server (namespace: %s)", node_.getNamespace().c_str());
      return;
    }else{
        ROS_WARN_STREAM("ImageStormApp:  Asset path is [" << WallBall::s_AssetPath << "]");
    }

    std::cout << "WallBall asset path: " << WallBall::s_AssetPath << "\n" << std::flush;

    // SET UP Qt
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setFocusPolicy(Qt::StrongFocus);
    setAutoFillBackground(false);
    
    // START WALLBALL
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

bool lastUserFlags[NUM_USERS];
void WallBallWidget::updateUsers(){
    bool userFlags[NUM_USERS];
    for (int i = 0; i < NUM_USERS; ++i)
        userFlags[i] = false;

    AppUserMap::iterator uit;
    // for each user
    for(uit = users_.begin();uit!=users_.end();uit++){

        // will return the existing ball if user already exists for it
        GOBall* ball = WallBall::newBall(uit->first);
                
        AppUser appUser = uit->second;
        Eigen::Vector3d torso = appUser.jtPosByName("torso");
        Eigen::Vector3d lShoulder = appUser.jtPosByName("left_shoulder");
        Eigen::Vector3d rShoulder = appUser.jtPosByName("right_shoulder");
        Eigen::Vector3d head = appUser.jtPosByName("head");
        Eigen::Vector3d lHand = appUser.jtPosByName("left_hand");
        Eigen::Vector3d rHand = appUser.jtPosByName("right_hand");
        Eigen::Vector3d lElbow = appUser.jtPosByName("left_elbow");
        Eigen::Vector3d rElbow = appUser.jtPosByName("right_elbow");

        double leanOffset =lElbow.x() - lHand.x();
        //double leanOffset = abs(torso.x() - rHand.x()) - abs(torso.x() - lHand.x());
        //double leanOffset = torso.x() - (lShoulder.x() + rShoulder.x())/2.0f;
 
        /*if (abs(leanOffset) < 25)
            leanOffset = 0.0f;
        else if (abs(leanOffset) < 45)
            leanOffset = leanOffset / abs(leanOffset) * 30.0f;
        else
        leanOffset = leanOffset / abs(leanOffset) * 50.0f;*/

        InputManager::SetMoveDirection(uit->first, leanOffset/500.0f);

        //bool jump = lHand.y() > head.y() && rHand.y() > head.y();
        bool jump = rHand.y() > rShoulder.y();
        InputManager::SetJump(uit->first, jump);

        userFlags[uit->first] = true;
    }

    for (int i = 0; i < NUM_USERS; ++i) {
        // if a user left, remove their ball from the game
        if (userFlags[i] == false && lastUserFlags[i] == true) {
            WallBall::removeBall(i);
        }

        lastUserFlags[i] = userFlags[i];
    }
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
    this->updateUsers();
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

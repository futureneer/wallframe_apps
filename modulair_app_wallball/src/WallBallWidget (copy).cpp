#include "WallBallWidget.h"

#include <Managers/GraphicsManager.h>

#include <iostream>

#include <QTimer>
#include <QDialog>
#include <QHBoxLayout>
#include <pthread.h>
using namespace std;

//using lair::WallBallWidget;

//namespace lair {

bool WallBallWidget::s_RunPhysics = false;

pthread_t s_PhysicsThread;
void* WallBallWidget::physicsThreadMethod(void* data) {
    cout << "physicsThreadMethod()\n" << flush;
    while (s_RunPhysics) {
    	WallBall::tickPhysics();
    }
    return 0;
}

WallBallWidget::WallBallWidget(QWidget *parent) :
    QWidget(parent)
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

/*
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

    //} // namespace lair

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
    AppBase(parent)
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

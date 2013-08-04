// WallBall.h

#include <WallBall.h>

#include <iostream>

GOBall** WallBall::s_GOBalls = new GOBall*[NUM_BALLS];

bool WallBall::s_Running = false;

void WallBall::start() {
	WallBall::s_Running = true;
	
	GraphicsManager::singleton();
    PhysicsManager::singleton();
	InputManager::singleton();
	GameObjectManager::singleton();
	
	WallBall::setupLevel();
}

void WallBall::start(QWidget* qWidget) {
   	GraphicsManager::setQWidget(qWidget);
	WallBall::start();
}

void WallBall::setupLevel() {
	WallBall::s_GOBalls[0] = new GOBall(Qt::Key_W,Qt::Key_A,Qt::Key_D);
	WallBall::s_GOBalls[1] = new GOBall(Qt::Key_I,Qt::Key_J,Qt::Key_L);
	
	GameObject* goStar = new GOStar();
	
	GameObject* levelFrame = new GOLevelFrame();
	GameObject* bottomRamp = new GOBottomRamp();
	
	GameObject* platform1 = new GOPlatform();
	platform1->move(0.0f, 14.0f);
	GameObject* platform2 = new GOPlatform();
	platform2->move(0.0f, 28.0f);
	
	GameObject* sideRamp1 = new GOSideLRamp();
	sideRamp1->move(20.0f, 12.0f);
	GameObject* sideRamp2 = new GOSideLRamp();
	sideRamp2->move(20.0f, 26.0f);
	
	GameObject* sideRamp3 = new GOSideRRamp();
	sideRamp3->move(-20.0f, 12.0f);
	GameObject* sideRamp4 = new GOSideRRamp();
	sideRamp4->move(-20.0f, 26.0f);
	
	// First floor spawn points
	GameObject* spawnPoint0 = new GOSpawnPoint(0.0f, 12.0f);
	GameObject* spawnPoint1 = new GOSpawnPoint(16.0f, 7.0f);
	GameObject* spawnPoint2 = new GOSpawnPoint(-16.0f, 7.0f);
	GameObject* spawnPoint3 = new GOSpawnPoint(27.0f, 10.5f);
	GameObject* spawnPoint4 = new GOSpawnPoint(-27.0f, 10.5f);
	
	// Second floor spawn points
	GameObject* spawnPoint5 = new GOSpawnPoint(0.0f, 27.0f);
	GameObject* spawnPoint6 = new GOSpawnPoint(16.0f, 20.0f);
	GameObject* spawnPoint7 = new GOSpawnPoint(-16.0f, 20.0f);
	GameObject* spawnPoint8 = new GOSpawnPoint(27.0f, 25.5f);
	GameObject* spawnPoint9 = new GOSpawnPoint(-27.0f, 25.5f);
	
	// Third floor spawn points
	GameObject* spawnPoint10 = new GOSpawnPoint(0.0f, 40.5f);
	GameObject* spawnPoint11 = new GOSpawnPoint(16.0f, 34.0f);
	GameObject* spawnPoint12 = new GOSpawnPoint(-16.0f, 34.0f);
	GameObject* spawnPoint13 = new GOSpawnPoint(27.0f, 40.5f);
	GameObject* spawnPoint14 = new GOSpawnPoint(-27.0f, 40.5f);
	
	// Portals
	GOPortal::setupPortal(-33.0f, 6.5f, 33.0f, 37.5f);
	GOPortal::setupPortal(-33.0f, 23.5f, 33.0f, 23.5f);
	GOPortal::setupPortal(-33.0f, 37.5f, 33.0f, 6.5f);
}

bool WallBall::tick() {
	return WallBall::tickPhysics() && WallBall::tickGraphics();
}

bool WallBall::tickPhysics() {
	if (//!InputManager::singleton()->tick() ||
		!PhysicsManager::singleton()->tick() ||
		!GameObjectManager::singleton()->tick() )
		return WallBall::end();

	return WallBall::s_Running;
}

bool WallBall::tickGraphics() {
	bool success = GraphicsManager::singleton()->tick();
	
	if ( !success ) {
		return WallBall::end();
	}

	return WallBall::s_Running;
}

bool WallBall::end() {

	GraphicsManager::shutdown();
	PhysicsManager::shutdown();

	return false;
}


GOBall* WallBall::getBall(int index) {
	return WallBall::s_GOBalls[index];
}

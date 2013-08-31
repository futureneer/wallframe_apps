// WallBall.h

#ifndef WALL_BALL
#define WALL_BALL

#include <irrlicht/irrlicht.h>
#include <Box2D/Box2D.h>

#include <GameObject/GameObjectManager.h>
#include <GameObject/GameObject.h>
#include <GameObject/Component.h>

#include <GameObject/GOBall.h>
#include <GameObject/GOBlock.h>
#include <GameObject/GOPortal.h>
#include <GameObject/GOStar.h>
#include <GameObject/GOSpawnPoint.h>
#include <GameObject/LevelObjects.h>

#include <Components/CGraphicsObject.h>
#include <Components/CPhysicsObject.h>
#include <Components/CSpawnPoint.h>
#include <Components/CController.h>
#include <Components/CKeyboardController.h>

#include <Managers/GraphicsManager.h>
#include <Managers/PhysicsManager.h>
#include <Managers/InputManager.h>

#include <QWidget>

#define NUM_BALLS 6

class WallBall {

private:

	static bool s_Running;
	static bool end();

	//static GOBall** s_GOBalls;


public:
	
    static std::string s_AssetPath;

	static void start();
	static void start(QWidget* qWidget);
	static void setupLevel();
	
	static bool tick();
	static bool tickPhysics();
	static bool tickGraphics();
	
    static GOBall* newBall(int index);
    static bool removeBall(int index);
	static GOBall* getBall(int index);
    static std::map<int,GOBall*>* s_GOBallMap;
};

#endif

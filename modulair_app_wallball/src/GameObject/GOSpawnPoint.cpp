#define GO_SPAWN_POINT_DEBUG false

#include <GameObject/GOSpawnPoint.h>
#include <Components/CSpawnPoint.h>
#include <Components/CGraphicsObject.h>

#include <WallBall.h>

GOSpawnPoint::GOSpawnPoint(float xPos, float yPos) : GameObject() {

	addComponent(new CSpawnPoint(this, xPos, yPos));
	
	if (GO_SPAWN_POINT_DEBUG) {
		// make graphics component
		CGraphicsObject* graphicsComponent = new CGraphicsObject(this, (WallBall::s_AssetPath + "/cube.3ds").c_str(), (WallBall::s_AssetPath + "/color8.bmp").c_str(), vector3df(0.3f, 0.3f, 0.3f) );
		addComponent(graphicsComponent);
		graphicsComponent->setPosition(xPos, yPos, 0.0f);
		graphicsComponent->setRotation(45.0f);
	}
}

GOSpawnPoint::~GOSpawnPoint() {
	
}


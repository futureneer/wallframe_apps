
#include <GameObject/GOStar.h>

#include <Components/CGraphicsObject.h>
#include <Components/CSpawnPoint.h>

#include <iostream>
#include <math.h>

#include <WallBall.h>

GOStar::GOStar() : GameObject() {
	CGraphicsObject* graphicsComponent = new CGraphicsObject(this, "WallBall/media/star.3ds", "WallBall/media/color2.bmp", vector3df(1.0f, 1.0f, 1.0f) );
	addComponent(graphicsComponent);
}

GOStar::~GOStar() {
	
}

void GOStar::start() {
	GameObject::start();
	spawn();
}

#define COLLECT_DISTANCE 2.0f
void GOStar::tick() {
	GameObject::tick();
	
	CGraphicsObject* graphicsComponent = (CGraphicsObject*)getComponent(CGraphicsObject::classTypeID());
	graphicsComponent->spin(1.0f);
	
	// check all balls to see if one has scored
	for (int i = 0; i < NUM_BALLS; i++) {
		GOBall* goBall = WallBall::getBall(i);
		b2Vec2 position = goBall->getPosition();
		if ( sqrt(pow(m_Y - position.y,2.0f) + pow(m_X - position.x,2.0f)) < COLLECT_DISTANCE) {
			spawn();
		}
	}
}

void GOStar::spawn() {

	CSpawnPoint* spawnPoint = CSpawnPoint::getRandomSpawnPoint();
	m_X = spawnPoint->getX();
	m_Y = spawnPoint->getY();
	
	CGraphicsObject* graphicsComponent = (CGraphicsObject*)getComponent(CGraphicsObject::classTypeID());
	graphicsComponent->setPosition(m_X, m_Y, 0.0f);
}
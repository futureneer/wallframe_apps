
#include <GameObject/GOBlock.h>

#include <Components/CGraphicsObject.h>
#include <Components/CPhysicsObject.h>

#include <WallBall.h>

GOBlock::GOBlock(float xPos, float yPos, float xScale, float yScale) : GameObject() {
	init(xPos, yPos, xScale, yScale, 0.0f, (WallBall::s_AssetPath + "/color1.bmp").c_str());
}

GOBlock::GOBlock(float xPos, float yPos, float xScale, float yScale, float rotation) : GameObject() {
	init(xPos, yPos, xScale, yScale, rotation, (WallBall::s_AssetPath + "/color1.bmp").c_str());
}

GOBlock::GOBlock(float xPos, float yPos, float xScale, float yScale, float rotation, const char* matFile) : GameObject() {
	init(xPos, yPos, xScale, yScale, rotation, matFile);
}

GOBlock::~GOBlock() {
	
}

const float PI = 3.14159265359f;

void GOBlock::init(float xPos, float yPos, float xScale, float yScale, float rotation, const char* matFile) {
	
	// make graphics component
	CGraphicsObject* graphicsComponent = new CGraphicsObject(this, (WallBall::s_AssetPath + "/cube.3ds").c_str(), matFile, vector3df(xScale, yScale, 1.0f) );
	addComponent(graphicsComponent);
	graphicsComponent->setPosition(xPos, yPos, 0.0f);
	graphicsComponent->setRotation(rotation);
	
	// make physics component
	b2PolygonShape box;
	box.SetAsBox(xScale, yScale);
	
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &box;
	fixtureDef.density = 0.0f;
	fixtureDef.friction = 0.3f;
	
	b2BodyDef bodyDef;
	bodyDef.position.Set(xPos, yPos);
	bodyDef.angle = rotation * PI / 180.0f;
	
	addComponent(new CPhysicsObject(this, &bodyDef, &fixtureDef));
}

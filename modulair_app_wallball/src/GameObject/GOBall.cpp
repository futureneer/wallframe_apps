
#include <GameObject/GOBall.h>

#include <Components/CGraphicsObject.h>
#include <Components/CPhysicsObject.h>
#include <Components/CSpawnPoint.h>
#include <Components/CController.h>
#include <Components/CKeyboardController.h>
#include <Components/CKinectController.h>

#include <Qt>
#include <math.h>

#include <WallBall.h>

#include <iostream>

bool GOBall::USE_KINECT = true;

GOBall::GOBall() : GameObject() {

	init(Qt::Key_W,Qt::Key_A,Qt::Key_D,0);
}

GOBall::GOBall(int playerNum) : GameObject() {

	init(Qt::Key_W,Qt::Key_A,Qt::Key_D,playerNum);
}

GOBall::GOBall(int jumpKey, int leftKey, int rightKey) : GameObject() {

	init(jumpKey,leftKey,rightKey,0);
	
}

GOBall::GOBall(int jumpKey, int leftKey, int rightKey, int playerNum) : GameObject() {

	init(jumpKey,leftKey,rightKey,playerNum);
	
}

void GOBall::init(int jumpKey, int leftKey, int rightKey, int playerNum) {
	
    this->m_Alive = true;

    this->m_Score = 0;
    this->m_ScoreObject = new GOScore(playerNum);

    // make graphics component
    const char* texturePath = (WallBall::s_AssetPath + "/color0.bmp").c_str();
    switch (playerNum) {
    case 0: texturePath = (WallBall::s_AssetPath + "/color0.bmp").c_str(); break;
    case 1: texturePath = (WallBall::s_AssetPath + "/color2.bmp").c_str(); break;
    case 2: texturePath = (WallBall::s_AssetPath + "/color3.bmp").c_str(); break;
    case 3: texturePath = (WallBall::s_AssetPath + "/color4.bmp").c_str(); break;
    case 4: texturePath = (WallBall::s_AssetPath + "/color6.bmp").c_str(); break;
    case 5: texturePath = (WallBall::s_AssetPath + "/color8.bmp").c_str(); break;
    }

    //char tempString[strlen(texturePath)];
    //char *tempString = (char *)calloc(strlen(texturePath), sizeof(char));
    //strcpy(tempString, texturePath);

    std::cout << "GOBALL: Texture filename: " << texturePath << "\n" << std::flush;
	addComponent(new CGraphicsObject(this, (WallBall::s_AssetPath + "/sphere.3ds").c_str(), texturePath));
	std::cout << "GOBALL: 2: Texture filename: " << texturePath << "\n" << std::flush;

    //free(tempString);

	// make controller component
    if (!GOBall::USE_KINECT)
        addComponent(new CKeyboardController(this, jumpKey, leftKey, rightKey));
    else
        addComponent(new CKinectController(this, playerNum));

	// make physics component
	b2CircleShape circle;
	circle.m_radius = 0.95f;
	
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &circle;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.4f;
	
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(0.0f, 20.0f);
	
	addComponent(new CPhysicsObject(this, &bodyDef, &fixtureDef));
}

const b2Vec2& GOBall::getPosition() {
	CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
	b2Body* body = physicsComponent->getBody();
	return body->GetPosition();
}

void GOBall::kill() {
    m_Alive = false;
    CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
    physicsComponent->kill();
    CGraphicsObject* graphicsComponent = (CGraphicsObject*)getComponent(CGraphicsObject::classTypeID());
    graphicsComponent->kill();
}

void GOBall::start() {
	GameObject::start();
	spawn();
}

void GOBall::tick() {
	GameObject::tick();
	
    if (!m_Alive)
        return;

    m_ScoreObject->updateScore(m_Score);

	jump();
	updateVelocity();
	
	// sync the position of the physics and graphics objects
	CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
	b2Body* body = physicsComponent->getBody();
	
	b2Vec2 position = body->GetPosition();
	float32 angle = body->GetAngle();

    if (position.y < -200)
        spawn();

	CGraphicsObject* graphicsComponent = (CGraphicsObject*)getComponent(CGraphicsObject::classTypeID());
	graphicsComponent->setPosition(position.x, position.y, 0.0f);
}

void GOBall::spawn() {

	CSpawnPoint* spawnPoint = CSpawnPoint::getRandomSpawnPoint();
	CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
	physicsComponent->setPosition(spawnPoint->getX(),spawnPoint->getY());
}

#define JUMP_CONST 22.5f
void GOBall::jump() {
	CController* controller = (CController*)getComponent(CController::classTypeID());
	if (controller->getJump()) {
		CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
		b2Body* body = physicsComponent->getBody();
		b2Vec2 velocity = body->GetLinearVelocity();
		body->SetLinearVelocity(b2Vec2(velocity.x, JUMP_CONST));
	}
}

#define VEL_MAX 18.0f
#define FORCE_CONST 25.0f
void GOBall::updateVelocity() {
	// get desired move velocity
	CController* controller = (CController*)getComponent(CController::classTypeID());
	float moveVelocity = controller->getMoveDirection() * VEL_MAX;
	
	// get actual velocity
	CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
	b2Body* body = physicsComponent->getBody();
	b2Vec2 position = body->GetPosition();
	b2Vec2 velocity = body->GetLinearVelocity();
	
	// calculate difference and product
	float product = velocity.x * moveVelocity;
	float difference = moveVelocity - velocity.x;
	
	// if moving, change move velocity
    /*if (moveVelocity != 0) {
        std::cout << "MOVING\n" << std::flush;
    }
    else {
        std::cout << "NOT MOVING\n" << std::flush;
        }*/

	if (moveVelocity != 0.0f &&
		(product < 0.0f || fabs(velocity.x) < fabs(moveVelocity) )) {
		float force = difference * FORCE_CONST;
		body->ApplyForce(b2Vec2(force, 0.0f), position);
	}
	else {
		float force = -velocity.x * FORCE_CONST * 0.2f;
		body->ApplyForce(b2Vec2(force, 0.0f), position);
	}
}

#define BOUNCE_IMPULSE 100.0f
void GOBall::onCollision(GameObject* other) {

	// if the other GameObject is another player, bounce
	if (other->getComponent(CController::classTypeID()) != 0) {
		b2Body* body = ((CPhysicsObject *)getComponent(CPhysicsObject::classTypeID()))->getBody();
		b2Body* otherBody = ((CPhysicsObject *)other->getComponent(CPhysicsObject::classTypeID()))->getBody();
		
		b2Vec2 position = body->GetPosition();
		b2Vec2 difference = position - otherBody->GetPosition();
		
//		std::cout << "Impulse 1 (" << difference.x << "," << difference.y <<")\n" << std::flush;
		
		difference.Normalize();
		difference *= BOUNCE_IMPULSE;
		
		body->ApplyLinearImpulse(difference, position);
		
//		std::cout << "Impulse 2 (" << difference.x << "," << difference.y <<")\n" << std::flush;
	}
}

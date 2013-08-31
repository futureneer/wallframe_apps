
#include <Managers/PhysicsManager.h>
#include <GameObject/GameObject.h>

#include <unistd.h>

PhysicsManager* PhysicsManager::s_Singleton = 0;

PhysicsManager* PhysicsManager::singleton() {
	if (PhysicsManager::s_Singleton == 0)
		PhysicsManager::s_Singleton = new PhysicsManager();
	return PhysicsManager::s_Singleton;
}

void PhysicsManager::shutdown() {
	if (PhysicsManager::s_Singleton != 0)
		delete PhysicsManager::s_Singleton;
	PhysicsManager::s_Singleton = 0;
}

PhysicsManager::PhysicsManager() {

	// make physics world
	b2Vec2 gravity(0.0f, -20.0f);
	m_World = new b2World(gravity);

	// make b2ContactListener
	ContactListener* contactListener = new ContactListener();
	m_World->SetContactListener(contactListener);
	
	// make ground physics object
/*	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, -10.0f);
	b2Body* groundBody = m_World->CreateBody(&groundBodyDef);

	b2PolygonShape groundBox;
	groundBox.SetAsBox(50.0f, 10.0f);
	groundBody->CreateFixture(&groundBox, 0.0f);*/
}

PhysicsManager::~PhysicsManager() {
	
	delete m_World;
}

#define TIME_STEP (1.0f / 60.0f)
#define TIME_SCALE (2.0f)
const int TIME_STEP_MICROSECONDS = (1000000 * TIME_STEP / TIME_SCALE);

#define VEL_ITERS 6
#define POS_ITERS 2

bool PhysicsManager::tick() {
	m_World->Step(TIME_STEP, VEL_ITERS, POS_ITERS);
    usleep(TIME_STEP_MICROSECONDS);
	
	return true;
}

b2Body* PhysicsManager::createBody(b2BodyDef* bodyDef) {
	return m_World->CreateBody(bodyDef);
}

void PhysicsManager::removeBody(b2Body* body) {
    m_World->DestroyBody(body);
}

void ContactListener::BeginContact(b2Contact* contact) {}

void ContactListener::EndContact(b2Contact* contact) {
	GameObject* gameObjectA = (GameObject*)contact->GetFixtureA()->GetBody()->GetUserData();
	GameObject* gameObjectB = (GameObject*)contact->GetFixtureB()->GetBody()->GetUserData();
	
	gameObjectA->onCollision(gameObjectB);
	gameObjectB->onCollision(gameObjectA);
}

void ContactListener::PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {} 
void ContactListener::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {}


#include <Components/CPhysicsObject.h>

#include <Managers/PhysicsManager.h>
#include <GameObject/GameObject.h>

CPhysicsObject::CPhysicsObject(GameObject* gameObject, b2BodyDef* bodyDef, b2FixtureDef* fixtureDef) :
	Component(gameObject) {
	bodyDef->userData = gameObject;
	m_Body = PhysicsManager::singleton()->createBody(bodyDef);
	m_Body->CreateFixture(fixtureDef);
}

CPhysicsObject::~CPhysicsObject() {
	
}

int CPhysicsObject::s_TypeID = -1;
int CPhysicsObject::componentTypeID() {
	return classTypeID();
}

int CPhysicsObject::classTypeID() {
	if (s_TypeID < 0)
		s_TypeID = Component::nextTypeID();
	return s_TypeID;
}

void CPhysicsObject::start() {
	
}

void CPhysicsObject::tick() {
	
}

b2Body* CPhysicsObject::getBody() {
	return m_Body;
}

void CPhysicsObject::move(float dX, float dY) {
	b2Vec2 position = m_Body->GetPosition();
	float angle = m_Body->GetAngle();
	m_Body->SetTransform(b2Vec2(position.x + dX, position.y + dY), angle);
}

void CPhysicsObject::setPosition(float x, float y) {
	float angle = m_Body->GetAngle();
	m_Body->SetTransform(b2Vec2(x, y), angle);
}
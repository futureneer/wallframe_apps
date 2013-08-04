
#include <GameObject/GameObject.h>
#include <GameObject/GameObjectManager.h>
#include <Components/CGraphicsObject.h>
#include <Components/CPhysicsObject.h>

#include <iostream>

GameObject::GameObject() :
	m_HasStarted(false), m_Parent(0) {

	m_Children = new set<GameObject*>();
	m_Components = new map<int, Component*>();

	GameObjectManager::singleton()->registerGameObject(this);
	
	m_GoToNewPoint = false;
}

GameObject::~GameObject() {
	
	GameObjectManager::singleton()->unregisterGameObject(this);

	// delete all Component objects
	for (map<int, Component*>::iterator it = m_Components->begin();
		it != m_Components->end(); it++)
		delete it->second;
	
	delete m_Components;
}

bool GameObject::addComponent(Component* component) {

	bool success = true;

	int componentTypeID = component->componentTypeID();
	
	if (m_Components->find(componentTypeID) == m_Components->end())
		(*m_Components)[componentTypeID] = component;
	else
		success = false;

	return success;
}

Component* GameObject::getComponent(int componentTypeID) {
	
	if (m_Components->find(componentTypeID) != m_Components->end())
		return (*m_Components)[componentTypeID];
	
	return 0;
}

bool GameObject::addChild(GameObject* child) {
	if (m_Children->find(child) != m_Children->end())
		return false;
	m_Children->insert(child);
}

bool GameObject::removeChild(GameObject* child) {
	if (m_Children->find(child) == m_Children->end())
		return false;
	m_Children->erase(child);
}

void GameObject::start() {

	for (map<int, Component*>::iterator it = m_Components->begin(); 
		it != m_Components->end(); 
		it++) {
		Component* component = it->second;
		component->start();
	}

	for (set<GameObject*>::iterator it = m_Children->begin(); 
		it != m_Children->end(); 
		it++) {
		GameObject* child = *it;
		child->start();
	}
}

void GameObject::tick() {

	if (!m_HasStarted) {
		start();
		m_HasStarted = true;
	}
	
	if (m_GoToNewPoint) {
		
		CPhysicsObject* physicsComponent = (CPhysicsObject*)getComponent(CPhysicsObject::classTypeID());
		if (physicsComponent != 0) {
			b2Body* body = physicsComponent->getBody();
			float angle = body->GetAngle();
			body->SetTransform(m_NewPoint, angle);
		}
		
		m_GoToNewPoint = false;
	}

	for (map<int, Component*>::iterator it = m_Components->begin(); 
		it != m_Components->end(); 
		it++) {
		Component* component = it->second;
		component->tick();
	}

	for (set<GameObject*>::iterator it = m_Children->begin(); 
		it != m_Children->end(); 
		it++) {
		GameObject* child = *it;
		child->tick();
	}
}

void GameObject::move(float dX, float dY) {
	
	for (map<int, Component*>::iterator it = m_Components->begin(); 
		it != m_Components->end(); 
		it++) {
		Component* component = it->second;
		component->move(dX, dY);
	}

	for (set<GameObject*>::iterator it = m_Children->begin(); 
		it != m_Children->end(); 
		it++) {
		GameObject* child = *it;
		child->move(dX, dY);
	}
}

void GameObject::onCollision(GameObject* other) {}

void GameObject::setNewPoint(const b2Vec2& newPoint) {
	m_GoToNewPoint = true;
	m_NewPoint = newPoint;
}
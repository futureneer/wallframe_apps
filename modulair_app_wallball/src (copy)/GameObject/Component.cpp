
#include <GameObject/Component.h>
#include <GameObject/GameObject.h>

int Component::s_NextTypeID = 0;

Component::Component(GameObject* gameObject) : m_GameObject(gameObject) { }

Component::~Component() { }

int Component::nextTypeID() {
	return s_NextTypeID++;
}

void Component::start() {
	
}

void Component::tick() {
	
}

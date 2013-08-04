
#include <Components/CGraphicsObject.h>

#include <Managers/GraphicsManager.h>
#include <GameObject/GameObject.h>

CGraphicsObject::CGraphicsObject(GameObject* gameObject, path meshFilepath, path textureFilepath, vector3df scale) :
	Component(gameObject) {
	
	m_SceneNode = GraphicsManager::singleton()->getMeshSceneNode(meshFilepath, textureFilepath);
	m_SceneNode->setScale(scale);
    m_SceneNode->getMaterial(0).Shininess = 1.0f;
}

CGraphicsObject::~CGraphicsObject() {
	
}

void CGraphicsObject::makeShiny() {
    m_SceneNode->getMaterial(0).Shininess = 20.0f;
}

int CGraphicsObject::s_TypeID = -1;
int CGraphicsObject::componentTypeID() {
	return classTypeID();
}

int CGraphicsObject::classTypeID() {
	if (s_TypeID < 0)
		s_TypeID = Component::nextTypeID();
	return s_TypeID;
}

void CGraphicsObject::start() {
	
}

void CGraphicsObject::tick() {
	
}

void CGraphicsObject::setPosition(float x, float y, float z) {
	m_SceneNode->setPosition(vector3df(x,y,z));
}

void CGraphicsObject::setPosition(float x, float y) {
	vector3df position = m_SceneNode->getPosition();
	setPosition(x, y, position.Z);
}

void CGraphicsObject::setRotation(float rotation) {
	m_SceneNode->setRotation(vector3df(0.0f,0.0f,rotation));
}

void CGraphicsObject::spin(float degrees) {
	vector3df rotation = m_SceneNode->getRotation();
	m_SceneNode->setRotation(vector3df(rotation.X, rotation.Y + degrees, rotation.Z));
}

void CGraphicsObject::move(float dX, float dY) {
	vector3df position = m_SceneNode->getPosition();
	setPosition(position.X + dX, position.Y + dY, position.Z);
}

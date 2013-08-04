
#include <Components/CController.h>
#include <stdlib.h>
#include <GameObject/GameObject.h>

CController::CController(GameObject* gameObject) :
	Component(gameObject),
	m_MoveDirection(0.0f),
	m_Jump(false)
{
}

CController::~CController() {	
}

int CController::s_TypeID = -1;
int CController::componentTypeID() {
	return classTypeID();
}

int CController::classTypeID() {
	if (s_TypeID < 0)
		s_TypeID = Component::nextTypeID();
	return s_TypeID;
}

/*void CController::start() {
	
}

void CController::tick() {
	
}*/
	
float CController::getMoveDirection() {
	return m_MoveDirection;
}

bool CController::getJump() {
	bool retValue = m_Jump;
	m_Jump = false;
	return retValue;
}

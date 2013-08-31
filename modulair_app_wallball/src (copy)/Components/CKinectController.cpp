#include <Components/CKinectController.h>

#include <Managers/InputManager.h>
#include <stdlib.h>
#include <GameObject/GameObject.h>

#include <iostream>

CKinectController::CKinectController(GameObject* gameObject, int index) :
	CController(gameObject),
    m_Index(index),
	m_ReadyToJump(true)
{
}

CKinectController::~CKinectController() {	
}

void CKinectController::start() {
	m_MoveDirection = 0.0f;
}

void CKinectController::tick() {
	// set move direction
    //	m_MoveDirection = InputManager::isKeyDown(m_KeyLeft) * 1.0f + InputManager::isKeyDown(m_KeyRight) * -1.0f;
    //    std::cout << "Offset: " << InputManager::GetMoveDirection(m_Index) << "\n";
    m_MoveDirection = InputManager::GetMoveDirection(m_Index);

	// set jump
    if (!InputManager::GetJump(m_Index))
        m_ReadyToJump = true;
    else if (InputManager::GetJump(m_Index) && m_ReadyToJump == true) {
        m_Jump = true;
        m_ReadyToJump = false;
    }
}

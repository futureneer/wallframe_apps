#include <Components/CKeyboardController.h>

#include <Managers/InputManager.h>
#include <stdlib.h>
#include <GameObject/GameObject.h>

#include <iostream>

CKeyboardController::CKeyboardController(GameObject* gameObject, int keyJump, int keyLeft, int keyRight) :
	CController(gameObject),
	m_KeyJump(keyJump),
	m_KeyLeft(keyLeft),
	m_KeyRight(keyRight),
	m_ReadyToJump(true)
{
}

CKeyboardController::~CKeyboardController() {	
}

void CKeyboardController::start() {
	m_MoveDirection = 0.0f;
}

void CKeyboardController::tick() {
	// set move direction
	m_MoveDirection = InputManager::isKeyDown(m_KeyLeft) * 1.0f + InputManager::isKeyDown(m_KeyRight) * -1.0f;
	
	// set jump
	if (!InputManager::isKeyDown(m_KeyJump))
		m_ReadyToJump = true;
	else if (InputManager::isKeyDown(m_KeyJump) && m_ReadyToJump == true) {
		m_Jump = true;
		m_ReadyToJump = false;
	}
}

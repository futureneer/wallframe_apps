
#include <Managers/InputManager.h>

#include <GameObject/GOBall.h>

#include <iostream>

InputManager* InputManager::s_Singleton = 0;
InputManager* InputManager::singleton() {
	if (InputManager::s_Singleton == 0)
		InputManager::s_Singleton = new InputManager();
	return InputManager::s_Singleton;
}

void InputManager::shutdown() {
	if (InputManager::s_Singleton != 0)
		delete InputManager::s_Singleton;
	InputManager::s_Singleton = 0;
}

InputManager::InputManager() {
    int size = sizeof(m_KeyPressed)/sizeof(bool);
	for (int i = 0; i < size; ++i)
        m_KeyPressed[i] = false;

    for (int i = 0; i < 6; ++i)
        m_Jump[i] = false;
}

InputManager::~InputManager() {

}

bool InputManager::tick() {
	return false;
}

bool InputManager::isKeyDown(int key) {
	InputManager* inputManager = InputManager::singleton();
	return inputManager->m_KeyPressed[key % 1000];
}

void InputManager::setKeyDown(int key, bool value) {
	InputManager* inputManager = InputManager::singleton();
	inputManager->m_KeyPressed[key % 1000] = value;
}

void InputManager::SetMoveDirection(int index, float moveDirection) {
	InputManager* inputManager = InputManager::singleton();
    if (index >=0 && index < 6)
        inputManager->m_MoveDirection[index] = moveDirection;
}

void InputManager::SetJump(int index, bool jump) {
	InputManager* inputManager = InputManager::singleton();
    if (index >=0 && index < 6)
        inputManager->m_Jump[index] = jump;
}

float InputManager::GetMoveDirection(int index) {
	InputManager* inputManager = InputManager::singleton();
    if (index >= 0 && index < 6)
        return inputManager->m_MoveDirection[index];
    return 0.0f;
}

bool InputManager::GetJump(int index) {
	InputManager* inputManager = InputManager::singleton();
    if (index >= 0 && index < 6) {
        bool jump = inputManager->m_Jump[index];
        //inputManager->m_Jump[index] = false;
        return jump;
    }
    return false;
}

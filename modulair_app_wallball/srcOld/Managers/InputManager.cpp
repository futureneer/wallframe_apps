
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

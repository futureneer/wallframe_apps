
#include <GameObject/GameObjectManager.h>

#include <GameObject/GameObject.h>

GameObjectManager* GameObjectManager::s_Singleton = 0;

GameObjectManager* GameObjectManager::singleton() {

	if (GameObjectManager::s_Singleton == 0)
		GameObjectManager::s_Singleton = new GameObjectManager();

	return GameObjectManager::s_Singleton;
}

void GameObjectManager::shutdown() {
	if (GameObjectManager::s_Singleton != 0)
		delete GameObjectManager::s_Singleton;
	GameObjectManager::s_Singleton = 0;
}

GameObjectManager::GameObjectManager() :
	m_ShouldUnregister(true){
	
	m_GameObjects = new std::set<GameObject*>();
}

GameObjectManager::~GameObjectManager(){
	
	m_ShouldUnregister = false; // so that GameObjects don't try to unregister, thus changing the list during iteration

	for (std::set<GameObject*>::iterator it = m_GameObjects->begin();
		it != m_GameObjects->end(); it++)
		delete *it;

	delete m_GameObjects;
}

void GameObjectManager::registerGameObject(GameObject* gameObject) {

	if (m_GameObjects->find(gameObject) == m_GameObjects->end())
		m_GameObjects->insert(gameObject);

}

void GameObjectManager::unregisterGameObject(GameObject* gameObject) {

	if (!m_ShouldUnregister)
		return;

	if (m_GameObjects->find(gameObject) != m_GameObjects->end())
		m_GameObjects->erase(gameObject);

}

bool GameObjectManager::tick() {

	for (std::set<GameObject*>::iterator it = m_GameObjects->begin();
		it != m_GameObjects->end(); it++)
		(*it)->tick();
		
	return true;
}

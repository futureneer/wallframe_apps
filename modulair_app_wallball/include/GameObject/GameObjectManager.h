
#include <set>

class GameObject;

class GameObjectManager {

	static GameObjectManager* s_Singleton;

	std::set<GameObject*>* m_GameObjects;
	bool m_ShouldUnregister;

	GameObjectManager();
	~GameObjectManager();

public:

	static GameObjectManager* singleton();
	static void shutdown();

	void registerGameObject(GameObject* gameObject);
	void unregisterGameObject(GameObject* gameObject);

	bool tick();
};

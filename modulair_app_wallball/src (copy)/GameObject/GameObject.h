
#ifndef GAME_OBJECT
#define GAME_OBJECT

#include <map>
using std::map;

#include <set>
using std::set;

#include <Box2D/Box2D.h>

// Predicate to use for comparing components

class Component;

class GameObject {
	
private:

	GameObject* m_Parent;

	set<GameObject*>* m_Children;
	std::map<int, Component*>* m_Components;
	
	bool m_HasStarted;

protected:

	bool addComponent(Component*);
	
public:

	GameObject();
	~GameObject();
	
	bool addChild(GameObject* child);
	bool removeChild(GameObject* child);
	
	Component* getComponent(int componentTypeID);
	
	virtual void start();
	virtual void tick();
	
	void move(float dX, float dY);
	virtual void onCollision(GameObject* other);
	
	void setNewPoint(const b2Vec2& newPoint);
	
private:

	bool m_GoToNewPoint;
	b2Vec2 m_NewPoint;
};

#endif
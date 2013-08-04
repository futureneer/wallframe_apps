
#ifndef PHYSICS_MANAGER
#define PHYSICS_MANAGER

#include <Box2D/Box2D.h>

class PhysicsManager {

public:
	static PhysicsManager* singleton();
	static void shutdown();

	bool tick();
	
	b2Body* createBody(b2BodyDef* bodyDef);

private:

	static PhysicsManager* s_Singleton;

	PhysicsManager();
	~PhysicsManager();

	// _____________________________ PHYSICS MEMBERS

	b2World* m_World;
};

class ContactListener : public b2ContactListener {
 
public:
 
//    ContactListener();
//    ~ContactListener();
 
    virtual void BeginContact(b2Contact* contact);
    virtual void EndContact(b2Contact* contact);
    virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);    
    virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
 
};

#endif
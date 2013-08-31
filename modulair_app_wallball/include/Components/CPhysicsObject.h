
#ifndef C_PHYSICS_OBJECT
#define C_PHYSICS_OBJECT

#include <GameObject/Component.h>
#include <Box2D/Box2D.h>

// abstract class representing component
class CPhysicsObject : public Component {
	
public:

	CPhysicsObject(GameObject* gameObject, b2BodyDef* bodyDef, b2FixtureDef* fixtureDef);
	~CPhysicsObject();

	virtual int componentTypeID();
	static int classTypeID();
		
	virtual void start();
	virtual void tick();

    void kill();
	
	b2Body* getBody();
	
	virtual void move(float dX, float dY);
	void setPosition(float x, float y);
	
private:

	b2Body* m_Body;

	static int s_TypeID;
	
};

#endif

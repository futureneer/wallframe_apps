
#ifndef COMPONENT
#define COMPONENT

class GameObject;

// abstract class representing component
class Component {
	
public:

	Component(GameObject* gameObject);
	~Component();

	virtual int componentTypeID() = 0;

	virtual void start();
	virtual void tick();
	
	virtual void move(float dX, float dY) {};
	
protected:

	static int nextTypeID();
	
	GameObject* m_GameObject;
	
private:
	
	static int s_NextTypeID;
	
	
};

#endif

#ifndef C_CONTROLLER
#define C_CONTROLLER

#include <GameObject/Component.h>

class CController : public Component {
	
public:

	CController(GameObject* gameObject);
	~CController();

	virtual int componentTypeID();
	static int classTypeID();
		
	virtual void start() = 0;
	virtual void tick() = 0;
	
	float getMoveDirection();
	bool getJump();
	
protected:
	
	float m_MoveDirection;
	bool m_Jump;
	
private:

	static int s_TypeID;
	
};

#endif

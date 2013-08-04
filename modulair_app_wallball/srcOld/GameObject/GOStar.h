
#ifndef GO_STAR
#define GO_STAR

#include <GameObject/GameObject.h>

// Predicate to use for comparing components

class GOStar : public GameObject {
	
public:

	GOStar();
	~GOStar();
	
	virtual void start();
	virtual void tick();
	
private:

	void spawn();
	
	float m_X;
	float m_Y;
};

#endif
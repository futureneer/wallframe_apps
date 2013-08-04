
#include <GameObject/GameObject.h>
#include <GameObject/GOBlock.h>

// Predicate to use for comparing components

class GOPortal : public GOBlock {
	
public:

	static void setupPortal(float x1, float y1, float x2, float y2, const char* matFile = 0);

	GOPortal(float x, float y, float direction);
	GOPortal(float x, float y, float direction, const char* matFile);
	~GOPortal();
	
	virtual void onCollision(GameObject* other);
	
private:
	
    float m_X;
    float m_Y;

	float m_Direction;
	GOPortal* m_Target;
	void setTarget(GOPortal* portal);
};

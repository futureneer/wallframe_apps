
#include <GameObject/GameObject.h>

#include <Box2D/Box2D.h>

// Predicate to use for comparing components

class GOBall : public GameObject {
	
public:

	GOBall();
	GOBall(int jumpKey, int leftKey, int rightKey);
	
	void start();
	void tick();
	
	virtual void onCollision(GameObject* other);
	
	const b2Vec2& getPosition();
	
private:

	void init();

	void spawn();
	void jump();
	void updateVelocity();
	
};

#include <GameObject/GameObject.h>

#include <Box2D/Box2D.h>

class GOBall : public GameObject {
	
public:

	GOBall();
	GOBall(int playerNum);
	GOBall(int jumpKey, int leftKey, int rightKey);
	GOBall(int jumpKey, int leftKey, int rightKey, int playerNum);
	
	void start();
	void tick();
	
	virtual void onCollision(GameObject* other);
	
    int m_Score;

	const b2Vec2& getPosition();
	
private:

	void init(int jumpKey, int leftKey, int rightKey, int playerNum);

	void spawn();
	void jump();
	void updateVelocity();
	
};

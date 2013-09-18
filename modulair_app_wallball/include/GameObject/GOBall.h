#ifndef GO_BALL
#define GO_BALL

#include <GameObject/GameObject.h>
#include <GameObject/GOScore.h>

#include <Box2D/Box2D.h>

class GOBall : public GameObject {
	
public:

    static bool USE_KINECT;

	GOBall();
	GOBall(int playerNum);
	GOBall(int jumpKey, int leftKey, int rightKey);
	GOBall(int jumpKey, int leftKey, int rightKey, int playerNum);
	
	void start();
	void tick();
	
	virtual void onCollision(GameObject* other);
	
    void kill();

    int m_Score;

	const b2Vec2& getPosition();
	
private:

    bool m_Alive;
    GOScore *m_ScoreObject;

	void init(int jumpKey, int leftKey, int rightKey, int playerNum);

	void spawn();
	void jump();
	void updateVelocity();
	
};

#endif // GO_BALL

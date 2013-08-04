#ifndef C_SPAWN_POINT
#define C_SPAWN_POINT

#include <GameObject/Component.h>

#include <vector>

// abstract class representing component
class CSpawnPoint : public Component {
	
public:

	static CSpawnPoint* getRandomSpawnPoint();

	CSpawnPoint(GameObject* gameObject, float x, float y);
	~CSpawnPoint();

	virtual int componentTypeID();
	static int classTypeID();
		
	virtual void start();
	virtual void tick();
	
	void move(float dX, float dY);
	float getX();
	float getY();
	
private:

	static bool s_HasSeeded;
	
	static int s_TypeID;
	static std::vector<CSpawnPoint*> s_SpawnPoints;
	
	float m_X;
	float m_Y;
};

#endif

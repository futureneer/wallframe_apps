#ifndef GO_SPAWN_POINT
#define GO_SPAWN_POINT

#include <GameObject/GameObject.h>

// Predicate to use for comparing components

class GOSpawnPoint : public GameObject {
	
public:

	GOSpawnPoint(float xPos, float yPos);
	~GOSpawnPoint();
	
};

#endif // GO_SPAWN_POINT


#ifndef GO_BLOCK
#define GO_BLOCK

#include <GameObject/GameObject.h>

// Predicate to use for comparing components

class GOBlock : public GameObject {
	
public:

	GOBlock(float xPos, float yPos, float xScale, float yScale);
	GOBlock(float xPos, float yPos, float xScale, float yScale, float rotation);
	GOBlock(float xPos, float yPos, float xScale, float yScale, float rotation, const char* matFile);
	~GOBlock();
	
protected:
	
	void init(float xPos, float yPos, float xScale, float yScale, float rotation, const char* matFile);
	
};

#endif

#ifndef C_GRAPHICS_OBJECT
#define C_GRAPHICS_OBJECT

#include <irrlicht/irrlicht.h>
#include <GameObject/Component.h>

using namespace irr;
using core::vector3df;
using namespace scene;
using namespace io;

// abstract class representing component
class CGraphicsObject : public Component {
	
public:

	CGraphicsObject(GameObject* gameObject, path meshFilepath, path textureFilepath, vector3df scale = vector3df(1.0f, 1.0f, 1.0f));
	~CGraphicsObject();

	virtual int componentTypeID();
	static int classTypeID();
		
    void makeShiny();

	virtual void start();
	virtual void tick();
	
	void setPosition(float x, float y, float z);
	void setRotation(float rotation);
	void spin(float degrees);
	
	virtual void move(float dX, float dY);
	void setPosition(float x, float y);
	
private:

	IMeshSceneNode* m_SceneNode;

	static int s_TypeID;
	
};

#endif

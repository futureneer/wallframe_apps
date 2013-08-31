#ifndef C_KINECT_CONTROLLER
#define C_KINECT_CONTROLLER

#include <GameObject/Component.h>
#include <Components/CController.h>

class CKinectController : public CController {
	
public:

	CKinectController(GameObject* gameObject, int index);
	~CKinectController();
		
	virtual void start();
	virtual void tick();
	
private:
    int m_Index;
	
	bool m_ReadyToJump;
};

#endif

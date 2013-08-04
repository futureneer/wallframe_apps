#ifndef C_KEYBOARD_CONTROLLER
#define C_KEYBOARD_CONTROLLER

#include <GameObject/Component.h>
#include <Components/CController.h>

class CKeyboardController : public CController {
	
public:

	CKeyboardController(GameObject* gameObject, int keyJump, int keyLeft, int keyRight);
	~CKeyboardController();
		
	virtual void start();
	virtual void tick();
	
private:

	int m_KeyJump;
	int m_KeyLeft;
	int m_KeyRight;
	
	bool m_ReadyToJump;
};

#endif

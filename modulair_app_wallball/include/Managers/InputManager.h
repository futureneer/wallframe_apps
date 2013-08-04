
#ifndef INPUT_MANAGER
#define INPUT_MANAGER

//#include <irrlicht.h>

#include <set>

class InputManager {

public:

	static bool isKeyDown(int key);
	static void setKeyDown(int key, bool value);

	static InputManager* singleton();
	static void shutdown();

	bool tick();
	
private:

	static InputManager* s_Singleton;
	
	InputManager();
	~InputManager();
	
	bool m_KeyPressed[1000];
};

#endif
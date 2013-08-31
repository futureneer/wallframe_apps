
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
	
    static void SetMoveDirection(int index, float moveDirection);
    static void SetJump(int index, bool jump);

    static float GetMoveDirection(int index);
    static bool GetJump(int index);

private:

	static InputManager* s_Singleton;
	
	InputManager();
	~InputManager();
	
	bool m_KeyPressed[1000];

    float m_MoveDirection[6];
    bool m_Jump[6];
};

#endif

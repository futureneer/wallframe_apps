#ifndef _state_object_h
#define _state_object_h
namespace modulair{

#define STATE_ACTIVE 0
#define STATE_IDLE 1
#define STATE_WAITING 2
#define STATE_HIDDEN 3
#define STATE_DOCKED 4
#define STATE_INUSE 5
#define STATE_ENGAGED 6
#define STATE_FOCUSED 7
#define STATE_LIBRARY_IDLE 8
#define STATE_TOOLBELT_IDLE 9
#define STATE_TOOLBELT_WAITING 10
#define STATE_ACTIVATED 11
#define STATE_TIMEOUT 12
#define STATE_RESET 13
#define STATE_START 14
#define STATE_SUSPENDED 15
#define STATE_TRIGGERED 16
#define STATE_INACTIVE 17

#define STATE_WALL_IDLE 18
#define STATE_WALL_ACTIVE 19
#define STATE_WALL_ENGAGED 20
#define STATE_WALL_DOCKED 21
#define STATE_USER_IDLE 22
#define STATE_USER_ACTIVE 23
#define STATE_USER_DOCKED 24
#define STATE_USER_WAITING 25
#define STATE_USER_HOLSTERED 26
#define STATE_DOCKED_TOOL 27
#define STATE_HOLSTER_WAITING 28
#define STATE_MOVE_WAIT 29
#define STATE_CURSOR_WAITING 30

class StateObject{
public:
	StateObject()
	{
		this->curState = STATE_IDLE;
		this->prevState = curState;
	};
	~StateObject(){};
	void setState(int s)
	{
		this->prevState = curState;
		this->curState = s;
	}
	int state()
	{
		return curState;
	}
protected:
	int curState;
	int prevState;
private:
};
}
#endif
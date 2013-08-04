
#include <GameObject/GameObject.h>

// Predicate to use for comparing components

class GOLevelFrame : public GameObject {
public:
	GOLevelFrame();
};

class GOPlatform : public GameObject {
public:
	GOPlatform();
};

class GOBottomRamp : public GameObject {
public:
	GOBottomRamp();
};

class GOSideLRamp : public GameObject {
public:
	GOSideLRamp();
};

class GOSideRRamp : public GameObject {
public:
	GOSideRRamp();
};
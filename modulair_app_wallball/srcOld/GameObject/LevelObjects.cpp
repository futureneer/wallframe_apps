
#include <GameObject/GOBlock.h>
#include <GameObject/LevelObjects.h>

#include <iostream>

GOLevelFrame::GOLevelFrame() : GameObject() {
    std::cout << "GOLevelFrame()\n" << std::flush;
	this->addChild(new GOBlock(0.0f, 0.0f, 35.0f, 1.0f));
	this->addChild(new GOBlock(34.0f, 22.5f, 1.0f, 22.5f));
	this->addChild(new GOBlock(-34.0f, 22.5f, 1.0f, 22.5f));
	this->addChild(new GOBlock(0.0f, 45.0f, 35.0f, 1.0f));
}

GOPlatform::GOPlatform() : GameObject() {
	this->addChild(new GOBlock(0.0f, 5.5f, 5.0f, 0.5f));
}

GOBottomRamp::GOBottomRamp() : GameObject() {
	this->addChild(new GOBlock(0.0f, 5.5f, 5.0f, 0.5f));
	this->addChild(new GOBlock(9.15f, 3.0f, 5.0f, 0.5f, 330.0f));
	this->addChild(new GOBlock(-9.15f, 3.0f, 5.0f, 0.5f, -330.0f));
}

GOSideLRamp::GOSideLRamp() : GameObject() {
	this->addChild(new GOBlock(0.0f, 3.0f, 5.0f, 0.5f, -330.0f));
	this->addChild(new GOBlock(9.15f, 5.5f, 5.0f, 0.5f, 0.0f));
	this->addChild(new GOBlock(-9.15f, 0.5f, 5.0f, 0.5f, 0.0f));
}

GOSideRRamp::GOSideRRamp() : GameObject() {
	this->addChild(new GOBlock(0.0f, 3.0f, 5.0f, 0.5f, 330.0f));
	this->addChild(new GOBlock(9.15f, 0.5f, 5.0f, 0.5f, 0.0f));
	this->addChild(new GOBlock(-9.15f, 5.5f, 5.0f, 0.5f, 0.0f));
}

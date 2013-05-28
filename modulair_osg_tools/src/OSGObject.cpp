#include "OSGObject.h"
using namespace lair;

OSGObject::OSGObject(	QObject* parent,
						bool active,
						QList<osg::ref_ptr< osg::TextureRectangle > >* texPointer, 
						QString type) : 
						QObject(parent),
						OSGObjectWrapper(texPointer,type),  
						myParent(parent),
						isActive(active)
{
	engagedID = "null";
	_label = "null";
	engagedObject = NULL;
    connect( &_trigger, SIGNAL(timeout()), this, SLOT(signalTriggered()));
    connect( &_idleTimer, SIGNAL(timeout()), this, SLOT(moveToIdle()));
}

OSGObject::OSGObject() : OSGObjectWrapper(), QObject(NULL){}

bool OSGObject::hasInside(OSGObject* obj)
{
	osg::Vec3 c = this->getPosition();
	this->box_updated.set(  this->box.xMin()+c[0],this->box.yMin()+c[1],this->box.zMin(),
                            this->box.xMax()+c[0],this->box.yMax()+c[1],this->box.zMax());
	vct2 o = obj->getPos2D();
    double x = o[0];
    double y = o[1];

    if(	x > box_updated.xMin() &&
    	x < box_updated.xMax() &&
    	y > box_updated.yMin() &&
    	y < box_updated.yMax())
	{
		return true;
	}else{
		return false;
	}
}

void OSGObject::setLabel(QString l)
{
	_label = l;
}

void OSGObject::signalTriggered()
{
	// _trigger.stop();
	this->setIdle();
	this->setScale(savedScale);

	this->engagedID = "null";
	this->engagedObject = NULL;

	if(isActive){
		if(_label == "null"){
			Q_EMIT(triggered());
		}else{
			Q_EMIT(triggered());
			Q_EMIT(triggered(_label));
		}
	}
}

void OSGObject::setDisengaged()
{
    // _trigger.stop();
	this->setIdle();
	this->setScale(savedScale);
	this->engagedID = "null";
	this->engagedObject = NULL;
}

void OSGObject::setEngagedTo(QString oID, OSGObject* obj)
{
    this->engagedID = oID;
    this->engagedObject = obj;

    this->sM->setState(STATE_ENGAGED);
    // _trigger.start(4000);
}

void OSGObject::activeScaling(double pos)
{
	// TODO make proportional gain controlled
	double max = 650;
	double min = 375;
	if(pos>min){
		double scale = (pos-min)/(max-min);
		double minS = 1-(.15*scale);
		osg::Vec3 s = savedScale;
	    this->setScale(osg::Vec3(s[0]*minS,s[1]*minS,s[2]*minS));
	    if(pos>max){
	    	signalTriggered();
	    	std::cerr<<"Triggered ";
	    	if(_label!= "null") std::cerr<<_label.toStdString();
	    	std::cerr<<std::endl;
	    }
	}
}

void OSGObject::moveToFocus(int texID)
{
	this->setPosition(this->focusPoint);
    this->setTexture(texID);
    this->sM->setState(STATE_FOCUSED);
}

void OSGObject::moveToIdle()
{
	_idleTimer.stop();
	this->setPosition(this->idlePoint);
	this->setActive();
}

void OSGObject::waitAndMoveToIdle()
{
    this->setTexture(activeTexID);
	_idleTimer.start(3000);
}
#include "modulair_osg_tools/osg_object_base.h"
using namespace lair;

osg_object_base::osg_object_base() : osg::PositionAttitudeTransform()
{
    _visible = true;
     _attitude = osg::Vec3(0,0,0);
    // setAttitude(osg::Quat(_attitude[1],osg::Vec3(0,1,0),
    //                       _attitude[0],osg::Vec3(1,0,0),
    //                       _attitude[2],osg::Vec3(0,0,1)));getScaleg
    this->box.set(osg::Vec3(-1,-1,-1),osg::Vec3(1,1,1));
}

void osg_object_base::setHidden()
{
    if(_visible){
        savedScale = this->getScale();
        this->setScale(osg::Vec3(0,0,0));
        _visible = false;
    }
}

void osg_object_base::setVisible()
{
    if(!_visible){
        this->setScale(savedScale);
        _visible = true;
    }
}

bool osg_object_base::isVisible()
{
    return _visible;
}

void osg_object_base::setPos3DAbs(vct3 p)
{ 
    this->setPosition(osg::Vec3(p[0],p[1],p[2])); 
}

void osg_object_base::setPos3DAbs(osg::Vec3 p)
{ 
    this->setPosition(p); 
}

void osg_object_base::setPos3DRel(vct3 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0]+c[0],p[1]+c[1],p[2]+c[2]));
}

void osg_object_base::setPos3DRel(osg::Vec3 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0]+c[0],p[1]+c[1],p[2]+c[2]));
}

void osg_object_base::setPos3DRel(double x, double y, double z)
{
    std::cout<<"Desired: ("<<x<<","<<y<<","<<z<<")  ";
    osg::Vec3 c = this->getPosition();
    std::cout<<"Old: ("<<c[0]<<","<<c[1]<<","<<c[2]<<")  ";
    this->setPosition(osg::Vec3(x+c[0],y+c[1],z+c[2]));
    c = this->getPosition();
    std::cout<<"New: ("<<c[0]<<","<<c[1]<<","<<c[2]<<")"<<std::endl;
}

void osg_object_base::setPos2DAbs(vct2 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0],p[1],c[2]));
}   

void osg_object_base::setPos2DAbs(osg::Vec3 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0],p[1],c[2]));
}   

void osg_object_base::setPos2DRel(vct2 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0]+c[0],p[1]+c[1],c[2]));
}

void osg_object_base::setPos2DRel(osg::Vec3 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0]+c[0],p[1]+c[1],c[2]));
}

void osg_object_base::setPos2DRel(double x, double y)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(x+c[0],y+c[1],c[2]));
}

void osg_object_base::servoToPos(osg::Vec3 loc, double dur)
{
    servo_.startServo(this,loc,dur);
}

vct3 osg_object_base::getPos3D()
{
    osg::Vec3 c = this->getPosition();
    return vct3(c[0], c[1], c[2]);
}

osg::Vec3 osg_object_base::getPos3DVec3()
{
    vct3 tmp = this->getPos3D();
    return osg::Vec3(tmp[0],tmp[1],tmp[2]);
}

vct2 osg_object_base::getPos2D()
{
    osg::Vec3 c = this->getPosition();
    vct2 p(c[0],c[1]);
    return p;   
}

void osg_object_base::orbit(osg::Vec3 d)
{
    osg::Vec3 newAtt = _attitude + d;
    _attitude = newAtt;

    if(_attitude[0] > _pi/2) _attitude[0] = _pi/2;
    if(_attitude[0] < _pi/40) _attitude[0] = _pi/40;

    //std::cout<<"Attitude ( "<<_attitude[0]<<" , "<<_attitude[1]<<" , "<<_attitude[2]<<" )"<<std::endl;
    setAttitude(osg::Quat(_attitude[1],osg::Vec3(0,1,0),
                          _attitude[0],osg::Vec3(1,0,0),
                          _attitude[2],osg::Vec3(0,0,1)));
}

void osg_object_base::rotateRel(osg::Vec3 dt)
{
    osg::Vec3 newAtt = _attitude + dt;
    _attitude = newAtt;

    setAttitude(osg::Quat(_attitude[1],osg::Vec3(0,1,0),
                          _attitude[0],osg::Vec3(1,0,0),
                          _attitude[2],osg::Vec3(0,0,1)));    
}

void osg_object_base::rotateAbs(osg::Vec3 ang)
{
    osg::Vec3 newAtt = ang;
    _attitude = newAtt;

    setAttitude(osg::Quat(_attitude[1],osg::Vec3(0,1,0),
                          _attitude[0],osg::Vec3(1,0,0),
                          _attitude[2],osg::Vec3(0,0,1)));    
}

void osg_object_base::moveAndScale(osg::Vec3 pos, double scale)
{
    setPosition(pos);
    setScale(osg::Vec3(scale,scale,scale));
}

osg::Matrixd* osg_object_base::getWorldCoords()
{
    osg::Node* node = this;
    osg::ref_ptr<getWorldCoordOfNodeVisitor> ncv = new getWorldCoordOfNodeVisitor();
    if (node && ncv){
        node->accept(*ncv);
        return ncv->giveUpDaMat();
    }
    else{
        return NULL;
    }
}

osg::Vec3 osg_object_base::getWorldPosition()
{
    return getWorldCoords()->getTrans();
}

void osg_object_base::setWorldPosition(osg::Vec3 targ)
{
    osg::Vec3 p_w = getWorldPosition();
    osg::Vec3 diff = targ-p_w;
    setPosition(getPosition()+diff);
}

double osg_object_base::getDist(osg::ref_ptr<osg_object_base> a, osg::ref_ptr<osg_object_base> b)
{
    osg::Vec3 va,vb,v;
    double s;
    va = a->getWorldPosition();
    vb = b->getWorldPosition();
    v = vb-va;
    s = sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));
    return s;
}

bool osg_object_base::checkDist(osg::ref_ptr<osg_object_base> a, osg::ref_ptr<osg_object_base> b, double dist)
{
    osg::Vec3 va,vb,v;
    double s = 0;
    va = a->getWorldPosition();
    vb = b->getWorldPosition();
    v = vb-va;
    s = sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));

    if(s < dist){
        return true;
    }else{
        return false;
    }
}

void osg_object_base::setScaleAll(double scale)
{
    this->setScale(osg::Vec3(scale,scale,scale));
}

osg::BoundingBox osg_object_base::calcBB()
{
    osg::BoundingBox curB = box;
    osg::Vec3 min = curB._min;
    osg::Vec3 max = curB._max;
    osg::BoundingBox newB(min+this->getPosition(),max+this->getPosition());
    return newB;
}

// Servo Object //

ServoObject::ServoObject() : QObject(NULL)
{
      connect( &_servoTimer, SIGNAL(timeout()), this, SLOT(servo()));
}

void ServoObject::startServo(osg_object_base* obj, osg::Vec3 posDesired, double duration)
{
    _servoStart = obj->getPosition();
    _servoTarget = posDesired;
    _servoCount = 0;
    _tics = duration/.1;
    _servoTimer.start(100);
    _objPtr = obj;
}

void ServoObject::startServoAndScale(osg_object_base* obj, osg::Vec3 posDesired, double duration, double s)
{
    _servoStart = obj->getPosition();
    double sc = s/obj->getScale()[0];
    obj->setScale(osg::Vec3(sc,sc,sc));
    _servoTarget = posDesired;
    _servoCount = 0;
    _tics = duration/.1;
    _servoTimer.start(100);
    _objPtr = obj;
}

void ServoObject::servo()
{
    _servoCount++;
    osg::Vec3 tmp = _servoTarget - _servoStart;
    double percent = double(_servoCount)/double(_tics);
    osg::Vec3 inc = tmp * percent;
    osg::Vec3 des = _servoStart + inc;
    _objPtr->setPosition(des);
    if(_servoCount >= _tics){
        _servoTimer.stop(); 
        _servoCount = 0;     
    } 
} 


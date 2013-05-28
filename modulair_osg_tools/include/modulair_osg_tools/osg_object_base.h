#ifndef _osg_object_base_h
#define _osg_object_base_h
// QT Includes //
#include <QtCore/QTimer>
#include <QtGui>
// OSG Includes //
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgFX/Cartoon>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Notify>
#include <osg/TextureRectangle>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/TexMat>
#include <osg/Group>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osg/Material>
#include <osg/BlendFunc>
#include <osg/StateSet>
#include <osg/Light>
#include <osg/LightSource>
#include <iostream>
// Modulair
#include "modulair_osg_tools/state_object.h"

static const double _pi = 3.14159265;
using namespace std;

namespace modulair{

static void printv(osg::Vec3 v){
  std::cerr<<v[0]<<","<<v[1]<<","<<v[2]<<endl;
}

class OSGObjectBase;

typedef std::map<QString,OSGObjectBase*> ObjectMap;
typedef QList< osg::ref_ptr < OSGObjectBase > > ObjectList;
typedef QList< osg::ref_ptr < osg::TextureRectangle > > TextureList;
typedef QList< osg::ref_ptr < osg::Image> > TexImageList;

//======= getWorldCoordOfNodeVisitor =================
class getWorldCoordOfNodeVisitor : public osg::NodeVisitor
{
public:
  getWorldCoordOfNodeVisitor():
    osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
  {
    wcMatrix= new osg::Matrixd();
  }
  virtual void apply(osg::Node &node)
  {
    if (!done)
    {
      if ( 0 == node.getNumParents() ) // no parents
      {
        wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
        done = true;
      }
      traverse(node);
    }
  }
  osg::Matrixd* giveUpDaMat()
  {
    return wcMatrix;
  }
private:
  bool done;
  osg::Matrix* wcMatrix;
};


//========= Servo Camera ===============
class ServoCamera : public QObject
{
  Q_OBJECT
public:
  ServoCamera() : QObject(NULL)
  {
    connect( &_servoTimer, SIGNAL(timeout()), this, SLOT(servo()));
  }
  ~ServoCamera(){}
  void startServo(osg::Camera* cam, osg::Vec3 posDesired, double duration)
  {
    _cam = cam;
    _cam->getViewMatrixAsLookAt(_eye,_look,_up);

    _servoStart = _eye;
    _servoTarget = posDesired;
    _servoCount = 0;
    _tics = duration/.03;
    _servoTimer.start(33);
  }
  void translateCameraRel(osg::Camera* cam, osg::Vec3 pos_delta,
                          osg::Vec3& cur_eye,
                          osg::Vec3& cur_look,
                          osg::Vec3& cur_up)
  {
    osg::Vec3 new_eye = cur_eye+pos_delta;
    osg::Vec3 new_look = cur_look+pos_delta;
    cam->setViewMatrixAsLookAt(new_eye,new_look,cur_up);
    cur_eye = new_eye;
    cur_look = new_look;
  }
  void translateCameraAbs(osg::Camera* cam,
                          osg::Vec3 new_eye,
                          osg::Vec3 new_look,
                          osg::Vec3 new_up,
                          osg::Vec3& cur_eye,
                          osg::Vec3& cur_look,
                          osg::Vec3& cur_up)
  {
    cam->setViewMatrixAsLookAt(new_eye,new_look,new_up);
    cur_eye = new_eye;
    cur_look = new_look;
    cur_up = new_up;
  }
  void rotateCameraRel(    osg::Camera* cam,
                           osg::Vec3& cur_eye,
                           osg::Vec3& cur_look,
                           osg::Vec3& cur_up,
                           osg::Vec3& cam_start,
                           osg::Vec3& cam_off,
                           double theta)
  {

    osg::Quat R;
    R.makeRotate(theta,osg::Vec3(0,1,0),
                 0,osg::Vec3(1,0,0),
                 0,osg::Vec3(0,0,1));

    osg::Vec3 rotated = cur_look+R*cam_off;
    cur_eye = rotated;

    cam->setViewMatrixAsLookAt(   cur_eye,      // eye with new rotation
                                  cur_look,         // old look
                                  cur_up);          // old up
  }
private:
  QTimer _servoTimer;
  osg::Camera* _cam;
  osg::Vec3d _eye, _look, _up;
  osg::Vec3 _servoStart;
  osg::Vec3 _servoTarget;
  int _servoCount;
  int _tics;
Q_SIGNALS:
public Q_SLOTS:
  void servo()
  {
    _servoCount++;
    osg::Vec3 tmp = _servoTarget - _servoStart;
    double percent = double(_servoCount)/double(_tics);
    osg::Vec3 inc = tmp * percent;
    osg::Vec3d des = _servoStart + inc;


    _cam->setViewMatrixAsLookAt(  des, // eye
                                  osg::Vec3d( 0,0,0 ),  // look
                                  osg::Vec3d( 0,1,0 )); // up

    if(_servoCount >= _tics){
      _servoTimer.stop();
      _servoCount = 0;
    }
  }
};



//===============  ServoObject ====================
class ServoObject : public QObject
{
  Q_OBJECT

public:
  ServoObject();
  ~ServoObject(){}
  void startServo(OSGObjectBase* obj, osg::Vec3 posDesired, double duration);
  void startServoAndScale(OSGObjectBase* obj, osg::Vec3 posDesired,
                          double duration, double s);
private:
  // Timers //
  OSGObjectBase* _objPtr;
  QTimer _servoTimer;
  osg::Vec3 _servoStart;
  osg::Vec3 _servoTarget;
  int _servoCount;
  int _tics;
Q_SIGNALS:
public Q_SLOTS:
  void servo();
};



//================ OSGObjectBase ===================
class OSGObjectBase : public osg::PositionAttitudeTransform
{
public:
  OSGObjectBase();
  ~OSGObjectBase(){}
  // 3D Movement //
  void setPos3DAbs(osg::Vec3 p);
  void setPos3DRel(double dx, double dy, double dz);
  void setPos3DRel(osg::Vec3 p);
  void servoTo(osg::Vec3 pos, double duration);
  // 2D Movement //
  void setPos2DAbs(osg::Vec3 p);
  void setPos2DRel(osg::Vec3 p);
  void setPos2DRel(double dx, double dy);
  // Position //
  osg::Vec3 getPos3D();
  osg::Vec2 getPos2D();
  // Attitude //
  void orbit(osg::Vec3 d);
  void rotateRel(osg::Vec3 dt);
  void rotateAbs(osg::Vec3 ang);
  void moveAndScale(osg::Vec3 pos, double scale);
  // Visibility //
  void setHidden();
  void setVisible();
  void setScaleAll(double scale);
  bool isVisible();
  osg::Matrixd* getWorldCoords();
  osg::Vec3 getWorldPosition();
  void setWorldPosition(osg::Vec3 targ);
  virtual bool triggerBehavior(QString type){return true;}
  osg::Vec3& getAttitudeVector(){return _attitude;}
  void servoToPos(osg::Vec3 loc, double dur);
  static double getDist(osg::ref_ptr<OSGObjectBase> a, osg::ref_ptr<OSGObjectBase> b);
  static bool checkDist(osg::ref_ptr<OSGObjectBase> a, osg::ref_ptr<OSGObjectBase> b, double dist);
  osg::BoundingBox bbox(){return this->box;}
  osg::BoundingBox calcBB();
  osg::BoundingBox box;
protected:
  osg::BoundingSphere sph;
  osg::BoundingBox box_updated;
  osg::Vec3 savedScale;
  bool _visible;
  osg::Vec3 _attitude;
  ServoObject servo_;
};


} // lair namespace

#endif

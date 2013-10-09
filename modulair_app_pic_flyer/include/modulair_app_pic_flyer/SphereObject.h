#ifndef _sphereObject_h
#define _sphereObject_h

#include <modulair_osg_tools/osg_object_base.h>

namespace modulair {
  class SphereObject;

  typedef std::map<QString,SphereObject*> SphereMap;
  
  class SphereObject : public OSGObjectBase
  {
  public:
    SphereObject(double rad, osg::Vec3 center, osg::Vec4 color);
    ~SphereObject(){};
    void initGeometry(double radius);
    osg::Node* generateSphere(double radius);
    osg::Material* createSimpleMaterial(osg::Vec4 color);
    virtual bool triggerBehavior(QString type);
  protected:
  private:
    TextureList* _textures;
    osg::Sphere* _sph;
    osg::Geode* _sphGeode;
    osg::Material* _material;
    osg::Vec4 _idleColor; 
  };
}
#endif

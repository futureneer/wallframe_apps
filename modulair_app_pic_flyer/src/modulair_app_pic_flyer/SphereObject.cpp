#include <modulair_app_pic_flyer/SphereObject.h>

using namespace osg;
using namespace modulair;

SphereObject::SphereObject(double radius, osg::Vec3 center, osg::Vec4 color)
	: OSGObjectBase()
{
	// Set Bounding Box
    this->box.set(			-radius,-radius,-radius,
                  			radius,radius,radius);
    this->box_updated.set(  -radius,-radius,-radius,
                            radius,radius,radius);
    // Init Scale
    savedScale = this->getScale();
    _idleColor = color;
    // Init Geometry
    initGeometry(radius);
    this->setPos3DAbs(center);
  this->sph.set(center,radius);
}

void SphereObject::initGeometry(double radius)
{
	this->addChild(generateSphere(radius));
}

bool SphereObject::triggerBehavior(QString type)
{
    if(type == "active"){
        osg::Vec4 acolor(0,1,0,1);
        _material->setDiffuse(Material::FRONT, acolor);
        _material->setEmission(Material::FRONT, acolor);
        _material->setAlpha(Material::FRONT_AND_BACK,1.0);
        _sphGeode->getOrCreateStateSet()->setAttribute(_material);
        return true;
    }else if(type == "idle"){
        _material->setDiffuse(Material::FRONT, _idleColor);
        _material->setEmission(Material::FRONT, _idleColor);
        _material->setAlpha(Material::FRONT_AND_BACK,1.0);
        _sphGeode->getOrCreateStateSet()->setAttribute(_material);
        return true;
    }else if(type == "hidden"){
        _material->setAlpha(Material::FRONT_AND_BACK,0.0);
        _sphGeode->getOrCreateStateSet()->setAttribute(_material);
        return true;
    }else if(type == "ghost"){
        _material->setAlpha(Material::FRONT_AND_BACK,0.125);
        _sphGeode->getOrCreateStateSet()->setAttribute(_material);
        return true;
    }else{
        return false;
    }
}

osg::Node* SphereObject::generateSphere(double radius)
{
    _sph = new osg::Sphere(osg::Vec3(0,0,0),radius); 
    osg::ShapeDrawable* _sphDraw = new osg::ShapeDrawable(_sph); 
    _sphGeode = new osg::Geode(); 
    _sphGeode->addDrawable(_sphDraw); 
    osg::PositionAttitudeTransform* sphXForm = new osg::PositionAttitudeTransform();
    sphXForm->setPosition(osg::Vec3(0,0,0)); 
    sphXForm->addChild(_sphGeode); 
    // create white material
    _material = new Material();
    _material->setDiffuse(Material::FRONT, _idleColor);
    _material->setSpecular(Material::FRONT, Vec4(0.0, 0.0, 0.0, 1.0));
    _material->setAmbient(Material::FRONT,  Vec4(0.1, 0.1, 0.1, 1.0));
    _material->setEmission(Material::FRONT, Vec4(_idleColor[0]-.2,
                                                 _idleColor[1]-.2,
                                                 _idleColor[2]-.2,1));
    _material->setShininess(Material::FRONT, 10.0);
    _material->setAlpha(Material::FRONT_AND_BACK,_idleColor[3]);
    // Assign material
    //_sphGeode->getOrCreateStateSet()->setAttribute(_material);

    osg::StateSet* st = _sphGeode->getOrCreateStateSet();
    st->setAttributeAndModes(_material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    
    osg::BlendFunc* bf = new
    osg::BlendFunc(osg::BlendFunc::SRC_ALPHA,
    osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
    st->setAttributeAndModes(bf);
    
    return sphXForm; 
}

Material* SphereObject::createSimpleMaterial(Vec4 color)
{
    Material *material = new Material();
    material->setDiffuse(Material::FRONT,  Vec4(0.0, 0.0, 0.0, 1.0));
    material->setEmission(Material::FRONT, color);
    return material;
}

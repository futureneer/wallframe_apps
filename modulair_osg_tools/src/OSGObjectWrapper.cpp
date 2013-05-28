#include "OSGObjectWrapper.h"
using namespace lair;

OSGObjectWrapper::OSGObjectWrapper( QList<osg::ref_ptr< osg::TextureRectangle > >* texPointer, 
                                    QString type) 
    : osg::PositionAttitudeTransform()
{
    this->texturePointer = texPointer;
    this->type = type;
    if(type == "2-state"){
        datIndex = -1;
        startDatIndex = -1;        
    }else{
        datIndex = 0;
        startDatIndex = 0;
    }

    this->sM = new StateObject();
    sM->setState(STATE_IDLE);
}

OSGObjectWrapper::OSGObjectWrapper() : osg::PositionAttitudeTransform()
{
    this->sM = new StateObject();
    sM->setState(STATE_IDLE);
}

void OSGObjectWrapper::create(double xmin, double ymin, double zmin,
            double xmax, double ymax, double zmax,
            int inactiveID, int activeID)
{
    this->box.set(xmin,ymin,zmin,
                  xmax,ymax,zmax);
    this->box_updated.set(  xmin,ymin,zmin,
                            xmax,ymax,zmax);

    this->inactiveTexID = inactiveID;
    this->activeTexID = activeID;

    if(type == "2-state"){
        this->addChild(setupObject(this->inactiveTexID));
        this->textureState = "inactive";         
    }else{
        this->addChild(setupObject(0));
        this->textureState = "null";
    }

    savedScale = this->getScale();
}

void OSGObjectWrapper::swapImage()
{
    if(type == "2-state"){
        if(textureState == "inactive"){
            textureState = "active";
            setTexture(activeTexID);
        }else{
            textureState = "inactive";
            setTexture(inactiveTexID);
        }
    }
}

void OSGObjectWrapper::setActive()
{
    sM->setState(STATE_ACTIVE);
    setTexture(activeTexID);
    textureState = "active";
}

void OSGObjectWrapper::setIdle()
{
    sM->setState(STATE_IDLE);
    setTexture(inactiveTexID);
    textureState = "inactive";
}

void OSGObjectWrapper::setWaiting()
{
    sM->setState(STATE_WAITING);
}

void OSGObjectWrapper::setEngagedTo(QString oID,OSGObjectWrapper* obj)
{
    this->engagedObject = obj;
    this->engagedID = oID;
}

void OSGObjectWrapper::setDockedTo(QString oID,OSGObjectWrapper* obj)
{
    this->dockedObject = obj;
    this->dockedID = oID;
}

void OSGObjectWrapper::incrementImage()
{
    if(type == "sequence"){
        datIndex ++;
        int numImages = texturePointer->size()-2;
        if(datIndex>numImages)
            datIndex = 0;
        setTexture(datIndex);
    }
}

void OSGObjectWrapper::decrementImage()
{
    if(type == "sequence"){
        datIndex --;
        int numImages = texturePointer->size()-2;
        if(datIndex<0)
            datIndex = numImages;
        setTexture(datIndex);
    }
}

void OSGObjectWrapper::setHidden()
{
    if(sM->state() != STATE_HIDDEN){
        savedScale = this->getScale();
        this->setScale(osg::Vec3(0,0,0));
        sM->setState(STATE_HIDDEN);
    }
}

void OSGObjectWrapper::setVisible()
{
    if(sM->state() == STATE_HIDDEN){
        this->setScale(savedScale);
        sM->setState(STATE_IDLE);
    }
}

// Movement ///////////////////
void OSGObjectWrapper::setPos3DAbs(vct3 p){ this->setPosition(osg::Vec3(p[0],p[1],p[2])); }
void OSGObjectWrapper::setPos3DAbs(double x, double y, double z)
{ 
    this->setPosition(osg::Vec3(x,y,z)); 
}

void OSGObjectWrapper::setPos3DRel(vct3 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0]+c[0],p[1]+c[1],p[2]+c[2]));
}

void OSGObjectWrapper::setPos3DRel(double x, double y, double z)
{
    std::cout<<"Desired: ("<<x<<","<<y<<","<<z<<")  ";
    osg::Vec3 c = this->getPosition();
    std::cout<<"Old: ("<<c[0]<<","<<c[1]<<","<<c[2]<<")  ";
    this->asPositionAttitudeTransform()->setPosition(osg::Vec3(x+c[0],y+c[1],z+c[2]));
    c = this->getPosition();
    std::cout<<"New: ("<<c[0]<<","<<c[1]<<","<<c[2]<<")"<<std::endl;
    setCenter();
}

void OSGObjectWrapper::setPos2DAbs(vct2 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0],p[1],c[2]));
}   

void OSGObjectWrapper::setPos2DRel(vct2 p)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(p[0]+c[0],p[1]+c[1],c[2]));
}

void OSGObjectWrapper::setPos2DRel(double x, double y)
{
    osg::Vec3 c = this->getPosition();
    this->setPosition(osg::Vec3(x+c[0],y+c[1],c[2]));
    setCenter();
}

vct3 OSGObjectWrapper::getPos3D()
{
    osg::Vec3 c = this->getPosition();
    vct3 p(c[0],c[1],c[2]);
    return p;   
}

vct2 OSGObjectWrapper::getPos2D()
{
    osg::Vec3 c = this->getPosition();
    vct2 p(c[0],c[1]);
    return p;   
}

void OSGObjectWrapper::setCenter()
{
    osg::Vec3 c = this->getPosition();
    vct2 p(c[0],c[1]);
    this->center = p;   
}

void OSGObjectWrapper::reCenter()
{
    setPos2DAbs(center);  
}

void OSGObjectWrapper::circle(int runtime)
{
    vct2 p = center;
    vct2 pn(p[0]+(.005*sin(double(runtime)/10.0)),
            p[1]+(.005*cos(double(runtime)/10.0)));
    setPos2DAbs(pn);
}

osg::Node* OSGObjectWrapper::setupObject(int index)
{
    osg::Vec3 top_left(box.xMin(),box.yMax(),box.zMax());
    osg::Vec3 bottom_left(box.xMin(),box.yMin(),box.zMax());
    osg::Vec3 bottom_right(box.xMax(),box.yMin(),box.zMax());
    osg::Vec3 top_right(box.xMax(),box.yMax(),box.zMax());
    
    // create geometry
    osg::Geometry* geom = new osg::Geometry;
    
    osg::Vec3Array* vertices = new osg::Vec3Array(4);
    (*vertices)[0] = bottom_left;
    (*vertices)[1] = bottom_right;
    (*vertices)[2] = top_right;
    (*vertices)[3] = top_left;
    
    geom->setVertexArray(vertices);
    
    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f, 0.0f);
    (*texcoords)[1].set(1.0f, 0.0f);
    (*texcoords)[2].set(1.0f, 1.0f);
    (*texcoords)[3].set(0.0f, 1.0f);
    geom->setTexCoordArray(0,texcoords);
    
    osg::Vec3Array* normals = new osg::Vec3Array(1);
    (*normals)[0].set(0.0f,-1.0f,0.0f);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
    
    // disable display list so our modified tex coordinates show up
    geom->setUseDisplayList(false);
        
    osg::TexMat* texmat = new osg::TexMat;
    texmat->setScaleByTextureRectangleSize(true);
    
    // setup state
    osg::StateSet* state = geom->getOrCreateStateSet();
    state->setTextureAttributeAndModes(0, this->texturePointer->at(index), osg::StateAttribute::ON);
    state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
    
    // turn off lighting 
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    // turn on blending
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // install 'update' callback
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geom);
    
    return geode;
}

void OSGObjectWrapper::setTexture(int index)
{
    osg::Node* node_ref;
    osg::Geode* geode_ref;
    osg::Geometry* geometry_ref;

    node_ref = this->getChild(0);
    if(node_ref->asGroup() != 0){
        std::cout<<"Cannot set texture of a group."<<std::endl;
    }else{
        if(node_ref->asGeode() != 0){
            geode_ref = node_ref->asGeode();
        }else{
            std::cout<<"Not a geode"<<std::endl;
            return;   
        }
        if (geode_ref->getDrawable(0)->asGeometry() != 0){
            geometry_ref = geode_ref->getDrawable(0)->asGeometry();
        }else{
            std::cout<<"Not geometry"<<std::endl;
            return;
        }
        osg::StateSet* state = geometry_ref->getOrCreateStateSet();
        if(this->texturePointer->at(index) == NULL){
            std::cout<<"Texture is null"<<std::endl;
        }else{
            state->setTextureAttributeAndModes( 0, 
                                            this->texturePointer->at(index), 
                                            osg::StateAttribute::ON);
            state->setMode(GL_BLEND, osg::StateAttribute::ON);
    		state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    		state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);    
        }    
    }        
}
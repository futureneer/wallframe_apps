#include <osgText/String>
#include "modulair_osg_tools/osg_planar_object.h"

namespace modulair{

PlanarObject::PlanarObject(double xmin, double ymin, double zmin,
                           double xmax, double ymax, double zmax,
                		   QList<osg::ref_ptr< osg::TextureRectangle > >* tex, 
                           int texIndex, osg::Vec3 center)
	: OSGObjectBase()
{   
    _init = false;
	// Set Bounding Box
    this->box.set(			xmin,ymin,zmin,
                  			xmax,ymax,zmax);
    this->box_updated.set(  xmin,ymin,zmin,
                            xmax,ymax,zmax);
    // Init Scale
    savedScale = this->getScale();
    // Link Texture Pointer
    _textures = tex;
    _texIndex = texIndex;
    // Init Geometry
    initGeometry(_texIndex);
    this->setPos3DAbs(center);

}

PlanarObject::~PlanarObject()
{

}

PlanarObject::PlanarObject(double xmin, double ymin, double zmin,
                           double xmax, double ymax, double zmax,
                           QList<osg::ref_ptr< osg::TextureRectangle > >* tex,
                           osg::Vec3 center)
    : OSGObjectBase()
{
    _init = false;
    // Set Bounding Box
    this->box.set(          xmin,ymin,zmin,
                            xmax,ymax,zmax);
    this->box_updated.set(  xmin,ymin,zmin,
                            xmax,ymax,zmax);
    // Init Scale
    savedScale = this->getScale();
    this->setPos3DAbs(center);
    // Link Texture Pointer
    _textures = tex;

}

void PlanarObject::setID(QString id)
{
    this->_id = id;
}

void PlanarObject::setCollection(QString col)
{
    this->_collection = col;
}
void PlanarObject::applyNewTexture(int tindex)
{
    if(_init == false){
        _texIndex = tindex;
        // Init Geometry
        initGeometry(_texIndex);   
    }else{
        _texIndex = tindex;
        setTexture(_texIndex);
    }    
}

int PlanarObject::getTexIndex()
{
    return _texIndex;
}

void PlanarObject::setText(QString t)
{
    _textLabel = new osgText::Text();
    _textGeode = new osg::Geode();
    _textXform = new osg::PositionAttitudeTransform();
    _textVisible = true;

    _textGeode->addDrawable(_textLabel);
    _textXform->addChild(_textGeode);
    this->addChild(_textXform);

    _textLabel->setCharacterSize(10);
    _textLabel->setFont("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");
    _textLabel->setText(t.toStdString(), osgText::String::ENCODING_UTF8);

    _textLabel->setDrawMode(osgText::Text::TEXT);
    _textLabel->setAlignment(osgText::Text::CENTER_TOP);
    _textLabel->setPosition(osg::Vec3(0,-(95),0));
    _textLabel->setColor( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
    
}

void PlanarObject::setText(QString t, int sz, osg::Vec4 color, osg::Vec3 pos)
{
    _textLabel = new osgText::Text();
    _textGeode = new osg::Geode();
    _textXform = new osg::PositionAttitudeTransform();
    _textVisible = true;

    _textGeode->addDrawable(_textLabel);
    _textXform->addChild(_textGeode);
    this->addChild(_textXform);

    _textLabel->setCharacterSize(sz);
    _textLabel->setFont("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");
    _textLabel->setText(t.toStdString(), osgText::String::ENCODING_UTF8);
    
    _textLabel->setDrawMode(osgText::Text::TEXT);
    _textLabel->setAlignment(osgText::Text::CENTER_TOP);
    _textLabel->setPosition( pos );
    _textLabel->setColor( color );
    
}

void PlanarObject::showText()
{
    if(!_textVisible){
        _textXform->setScale(_textScale);
        _textVisible = true;
    }
}

void PlanarObject::hideText()
{
    if(_textVisible){
        _textScale = _textXform->getScale();
        _textXform->setScale(osg::Vec3(0,0,0));
        _textVisible = false;
    }    
}

bool PlanarObject::triggerBehavior(QString type)
{
    //std::cerr<<"Triggering: "<< type.toStdString()<<std::endl;
    if(type == "active"){
        setTexture(_texIndex+1);
        return true;
    }else if(type == "idle"){
        setTexture(_texIndex);
        return true;
    }else if(type == "hidden"){
        setTexture(_texIndex-1);
        return true;
    }else if(type == "-1"){
        setTexture(_texIndex-1);
        return true;
    }else if(type == "0"){
        setTexture(_texIndex);
        setVisible();
        return true;
    }else if(type == "1"){
        setTexture(_texIndex+1);
        setVisible();
        return true;
    }else if(type == "2"){
        setTexture(_texIndex+2);
        setVisible();
        return true;
    }else if(type == "3"){
        setTexture(_texIndex+3);
        setVisible();
        return true;
    }else{
        return false;
    }
}

void PlanarObject::initGeometry(int index)
{
	this->addChild(generatePlane(index));
    _init = true;
}

osg::ref_ptr<osg::Node> PlanarObject::generatePlane(int index)
{
    osg::Vec3 top_left(box.xMin(),box.yMax(),box.zMax());
    osg::Vec3 bottom_left(box.xMin(),box.yMin(),box.zMax());
    osg::Vec3 bottom_right(box.xMax(),box.yMin(),box.zMax());
    osg::Vec3 top_right(box.xMax(),box.yMax(),box.zMax());
    
    // create geometry
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
    (*vertices)[0] = bottom_left;
    (*vertices)[1] = bottom_right;
    (*vertices)[2] = top_right;
    (*vertices)[3] = top_left;
    
    geom->setVertexArray(vertices);
    
    osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f, 0.0f);
    (*texcoords)[1].set(1.0f, 0.0f);
    (*texcoords)[2].set(1.0f, 1.0f);
    (*texcoords)[3].set(0.0f, 1.0f);
    geom->setTexCoordArray(0,texcoords);
    
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array(1);
    (*normals)[0].set(0.0f,-1.0f,0.0f);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
    
    // disable display list so our modified tex coordinates show up
    geom->setUseDisplayList(false);
        
    osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
    texmat->setScaleByTextureRectangleSize(true);
    
    // setup state
    osg::StateSet* state = geom->getOrCreateStateSet();
    state->setTextureAttributeAndModes(0, this->_textures->at(index), osg::StateAttribute::ON);
    state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
    
    // turn off lighting 
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    // turn on blending
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // install 'update' callback
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);
    
    return geode;
}

void PlanarObject::setTransparency(double val)
{
    osg::ref_ptr<osg::Node> node_ref;
    osg::ref_ptr<osg::Geode> geode_ref;
    osg::ref_ptr<osg::Geometry> geometry_ref;

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
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
        (*colors)[0].set(1.0f,1.0f,1.0f,val);
        geometry_ref->setColorArray(colors);
        geometry_ref->setColorBinding(osg::Geometry::BIND_OVERALL);   
    }     
}

void PlanarObject::setTexture(int index)
{
    osg::ref_ptr<osg::Node> node_ref;
    osg::ref_ptr<osg::Geode> geode_ref;
    osg::ref_ptr<osg::Geometry> geometry_ref;

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
        if(this->_textures->at(index) == NULL){
            std::cout<<"Texture is null"<<std::endl;
        }else{
            state->setTextureAttributeAndModes( 0, 
                                            this->_textures->at(index), 
                                            osg::StateAttribute::ON);
            state->setMode(GL_BLEND, osg::StateAttribute::ON);
         state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
         state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);    
        }    
    }        
}

}

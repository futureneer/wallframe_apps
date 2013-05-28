#ifndef _osgobjectwrapper_h
#define _osgobjectwrapper_h

#include <QtCore/QTimer>
#include <QtGui>

#include <osgQt/GraphicsWindowQt>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
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
#include <iostream>
#include "StateObject.h"

#include <cisstVector.h>
namespace lair{
	
class OSGObjectWrapper : public osg::PositionAttitudeTransform
{
public:
    OSGObjectWrapper(QList<osg::ref_ptr< osg::TextureRectangle > >* texPointer, QString type);
    OSGObjectWrapper();
    ~OSGObjectWrapper(){}

    // Textures //
    void swapImage();
    void incrementImage();
    void decrementImage();

    // Movement //
    void setPos3DAbs(vct3 p);
    void setPos3DAbs(double dx, double dy, double dz);

    void setPos3DRel(vct3 p);
    void setPos3DRel(double dx, double dy, double dz);
    
    void setPos2DAbs(vct2 p);
    void setPos2DRel(vct2 p);
    void setPos2DRel(double dx, double dy);
    // Position //
    vct3 getPos3D();
    vct2 getPos2D();

    // Initialization //
    osg::Node* setupObject(int index);
    void setTexture(int index);
    void create(double xmin, double ymin, double zmin,
                double xmax, double ymax, double zmax,
                int inactiveID, int activeID);
    void setHidden();
    void setVisible();
    void setWaiting();
    void setEngagedTo(QString ID,OSGObjectWrapper* obj);
    void setDockedTo(QString ID,OSGObjectWrapper* obj);
    void setActive();
    void setIdle();
    void setCenter();
    void reCenter();
    vct2 center;
    void circle(int runtime);

    lair::StateObject* sM;
    lair::OSGObjectWrapper* engagedObject;
    QString engagedID;
    lair::OSGObjectWrapper* dockedObject;
    QString dockedID;

protected:
    osg::BoundingBox box;
    osg::BoundingBox box_updated;
    QList<osg::ref_ptr< osg::TextureRectangle > >* texturePointer;
    int inactiveTexID;
    int activeTexID;
    QString textureState;
    QString type;
    int datIndex;
    int startDatIndex;
    osg::Vec3 savedScale;
};
}

#endif /* _osgobjectwrapper_h */

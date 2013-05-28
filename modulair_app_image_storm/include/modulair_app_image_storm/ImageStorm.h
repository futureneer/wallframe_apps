#ifndef _ImageStorm_h
#define _ImageStorm_h
// QT //
#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <QtGui>
// OSG //
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
#include <osg/Material>
#include <osg/StateSet>
#include <osg/Light>
#include <osg/Depth>
#include <osg/LightSource>
#include <osg/Geode>
#include <osg/TexMat>
#include <osg/Group>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
// IOSTRAM //
#include <iostream>
// INTERNAL INCLUDES //
#include "AppBase.h"

#include "OSGObjectWrapper.h"
#include "StateObject.h"
#include "OSGObject.h"
#include "SkeletonObject.h"
#include "SphereObject.h"
#include "CubeObject.h"
#include "InteractionObject.h"
#include "PlanarObject.h"
#include "CursorObject.h"
#include "DockableObject.h"
#include "Holster.h"
#include "MoveTool.h"
#include "CameraTool.h"
#include "ActableObject.h"

namespace lair{

    class ImageStorm;
    class ImageStormKeyboardHandler : public osgGA::GUIEventHandler
    {
    public:
        bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa);
        virtual void accept(osgGA::GUIEventHandlerVisitor& v);
        virtual void setup(ImageStorm* appPt);
        ImageStorm* appPtr;
    };

    class ImageStorm : public AppBase, public osgViewer::Viewer
    {
        Q_OBJECT;
    public:
        ImageStorm( QWidget *par = NULL, QWidget *appManager = NULL, QString appID = "null", bool useKin = false);
        ~ImageStorm();
        
        // Qt Functions
        osgQt::GLWidget* addViewWidget( osg::Camera* camera, 
                                        osg::Node* scene );
        osg::Camera* createCamera( int x, int y, int w, int h,
                                   osgQt::GLWidget *QTObject,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        virtual void paintEvent( QPaintEvent* event ) { frame(); }
        // Methods
        void updateCursors();
        void updateEnvironment();
        void readConfigFile();
        void setImageDirectories();
        void LoadTextures();
        void collide();
        void updateUsers();
        
        // Camera Control Variables //
        osg::ref_ptr<osg::Camera> _camera;
        ServoCamera _camServo;
        osg::Vec3 _cameraOrbit;
        osg::Vec3 _cameraStart;

        // USERS //
        int numActiveUsers;

        bool activeUsers[12];
        int joint_increments[12];
        bool prev_activeUsers[12];

    private:


    public:

        ObjectList back_planes_;
        ObjectList usage_planes_;

        // Wrappers //
        osg::ref_ptr<OSGObjectBase> _envWrapper;
        osg::ref_ptr<OSGObjectBase> usage_wrapper_;
        osg::ref_ptr<OSGObjectBase> plane_wrapper_;
                                   
        // OPENGL and QT //
        osgQt::GLWidget* glWidget;
        osg::ref_ptr<osg::Group> root;
        
        // Variables
        QStringList assetPaths;
        QStringList imagePaths;
        QList<osg::ref_ptr< osg::TextureRectangle > > _assetTextures;
        QList<int> image_assignments;
        QList<osg::Vec3> plane_start_pos;

        QString assetDir;

        bool useKinect;

        static const double GUI_MAX_X =  1.0;
        static const double GUI_MIN_X = -1.0;
        static const double GUI_MAX_Y =  0.5;
        static const double GUI_MIN_Y = -0.5;
        
        int runtime;
        bool paused;
        int _envState;

    Q_SIGNALS:
        void showImgs(int node);
        void showThumbs();
        void removeAllActive();

    public Q_SLOTS:
        void config();
        void pause();
        void unpause();
        void resume();
        void suspend();
        void pullData(); 
        void increment();
        void recieveDiscreteGesture(QMap<int,int> events){};
        
    protected:
        // Timers //
        QTimer _timer;
        QTimer _dataTimer; 
        QTimer _delay;   
    };    
}
 
#endif
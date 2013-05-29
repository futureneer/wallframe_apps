#ifndef osg_app_h
#define osg_app_h 
// MODULAIR INCLUDES
#include <modulair_core/modulair_core.h>
#include <modulair_core/modulair_app_base.h>
#include <modulair_osg_tools/osg_object_base.h>
#include <modulair_osg_tools/osg_planar_object.h>
// APP SPECIFIC INCLUDES
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

namespace modulair{

  // 
  class OsgApp;
  class KeyboardHandler : public osgGA::GUIEventHandler
    {
    public:
        bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa);
        virtual void accept(osgGA::GUIEventHandlerVisitor& v);
        virtual void setup(OsgApp* appPt);
        OsgApp* appPtr;
    };

	class OsgApp : public ModulairAppBase, public osgViewer::Viewer{
    Q_OBJECT
  public:
    /* Constructors and Destructors*/
    OsgApp(QString app_name, ros::NodeHandle nh, int event_deque_size);
    ~OsgApp();
    /* ModulairAppBase Virtual Methods */
	bool build();
	bool start();
    bool stop();
    bool pause();
    bool resume();
    /*OsgApp specific members*/
    // Qt Functions
    osgQt::GLWidget* addViewWidget( osg::Camera* camera, 
                                    osg::Node* scene );
    osg::Camera* createCamera( int x, int y, int w, int h,
                               osgQt::GLWidget *QTObject,
                               const std::string& name="",
                               bool windowDecoration=false);
    virtual void paintEvent( QPaintEvent* event ) { frame(); }
    // Methods
    void updateEnvironment();
    void LoadTextures();
    // void collide();
    // void updateUsers();
        
    // Camera Control Variables //
    osg::ref_ptr<osg::Camera> _camera;
    ServoCamera _camServo;
    osg::Vec3 _cameraOrbit;
    osg::Vec3 _cameraStart;

    int num_planes;
    int plane_size_;
    int camera_position_offset;

    // USERS //
    int numActiveUsers;
    int images_per_user_;

    bool activeUsers[12];
    int joint_increments[12];
    bool prev_activeUsers[12];

    ObjectList back_planes_;
    // ObjectList usage_planes_;

    // Wrappers //
    osg::ref_ptr<OSGObjectBase> _envWrapper;
    // osg::ref_ptr<OSGObjectBase> usage_wrapper_;
    osg::ref_ptr<OSGObjectBase> plane_wrapper_;
                               
    // OPENGL and QT //
    osgQt::GLWidget* glWidget;
    osg::ref_ptr<osg::Group> root;
    
    // Variables
    QStringList assetPaths;
    QStringList imagePaths;
    QList<osg::ref_ptr< osg::TextureRectangle > > _assetTextures;
    // QList<int> image_assignments;
    QList<osg::Vec3> plane_start_pos;

    QString assetDir;

    // bool useKinect;

    // static const double GUI_MAX_X =  1.0;
    // static const double GUI_MIN_X = -1.0;
    // static const double GUI_MAX_Y =  0.5;
    // static const double GUI_MIN_Y = -0.5;
    
    // int runtime;
    bool paused;
    // int _envState;

  Q_SIGNALS:
    // void showImgs(int node);
    // void showThumbs();
    // void removeAllActive();

  public Q_SLOTS:   
    // void config();
    // void pause();
    // void unpause();
    // void resume();
    // void suspend();
    void updateApp(); 
        
  protected:
    // Timers //
    QTimer _timer;
    QTimer _dataTimer; 
    // QTimer _delay; 
	};

}

#endif


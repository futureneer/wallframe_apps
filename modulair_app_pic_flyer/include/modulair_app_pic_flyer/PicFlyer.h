#ifndef _PicFlyer_H
#define _PicFlyer_H
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

#include <iostream>

#include <wallframe_core/wallframe_app_base_qt.h>
#include <modulair_osg_tools/osg_object_base.h>
#include <modulair_osg_tools/osg_planar_object.h>

/* Old includes.
#include "AppBase.h"
#include "OSGObjectWrapper.h"
#include "StateObject.h"
#include "OSGObject.h"
#include "SkeletonObject.h"
#include "SphereObject.h"
#include "CubeObject.h"
#include "InteractionObject.h"
#include "PlanarObject.h"
#include "TexturedPlane.h"
#include "CursorObject.h"
#include "DockableObject.h"
#include "Holster.h"
#include "MoveTool.h"
#include "CameraTool.h"
#include "ActableObject.h"
*/

//MODEL FILES
#include <modulair_app_pic_flyer/Model/PictureCollectionFactory.h>
#include <modulair_app_pic_flyer/Model/PictureCollection.h>
#include <modulair_app_pic_flyer/Model/ManuscriptCollection.h>
#include <modulair_app_pic_flyer/Model/ManuscriptCollectionFactory.h>
#include <modulair_app_pic_flyer/Model/Manuscript.h>
#include <modulair_app_pic_flyer/Model/DisplayImage.h>
#include <modulair_app_pic_flyer/Model/Opening.h>
#include <modulair_app_pic_flyer/Model/CollectionStore.h>

namespace PicFlyerApp {

    //Constants
    static const int WKSP_MIN_X_PIC = -1500; 
    static const int WKSP_MAX_X_PIC = 1500; 
    static const int WKSP_MIN_Y_PIC = -800; 
    static const int WKSP_MAX_Y_PIC = 400; 
    static const int WKSP_MIN_Z_PIC = 500; 
    static const int WKSP_MAX_Z_PIC = 4000;

    static const int GUI_MIN_X_PIC = -1500;
    static const int GUI_MAX_X_PIC = 1500;
    static const int GUI_MIN_Y_PIC = -1200;
    static const int GUI_MAX_Y_PIC = 1200;
    static const int GUI_MIN_Z_PIC = 800;
    static const int GUI_MAX_Z_PIC = 1600;
    // static const double KINECT_ROT_ANG = 3.14159265/8;

    enum envState_t {   ENV_STATE_INDEX,
                        ENV_STATE_ARRAY,
                        ENV_STATE_MATRIX,
                        ENV_STATE_IMAGE,
                        // ENV_STATE_WAITING_LEAVE,
                        ENV_STATE_WAITING,
                        ENV_STATE_WAITING_SELECTED,
                        ENV_STATE_WAITING_BACK,
                        ENV_STATE_TURN,
                        ENV_STATE_SELECT,
                        ENV_STATE_BACK,
                        // ENV_STATE_LEAVE,
                        ENV_STATE_IMAGE_ZOOM,
                        ENV_STATE_WAITING_MATRIX,
                        ENV_STATE_WAITING_ARRAY,
                        ENV_STATE_IMAGE_PAN,
                        //ENV_STATE_PAGE_COLLECTION,
                        ENV_STATE_PAGE_TURNER};

    enum previousState_t {  INDEX_SELECTION,
                            MANUSCRIPT_SELECTION,
                            COLLECTION_SELECTION,
                            OPENING_THUMBNAILS,
                            STANDARD_THUMBNAILS,
                            PAGE_TURNER,
                            LARGE_IMAGE,
                            LARGE_IMAGE_T};

    enum turn_t {   TURN_LEFT,
                    TURN_RIGHT};


    class PicFlyer;
    class PicFlyerKeyboardHandler : public osgGA::GUIEventHandler
    {
    public:
        bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa);
        virtual void accept(osgGA::GUIEventHandlerVisitor& v);
        virtual void setup(PicFlyer* appPt);
        PicFlyer* appPtr;
    };

    class PicFlyer : public wallframe::WallframeAppBaseQt, public osgViewer::Viewer
    {
        Q_OBJECT;
    public: 
        PicFlyer(QString app_name, ros::NodeHandle nh, int event_deque_size);
        ~PicFlyer(){}
        
        // Functions
        // osg::Node* createRectangle(osg::BoundingBox& bb, int index);
        osgQt::GLWidget* addViewWidget( osg::Camera* camera, 
                                        osg::Node* scene );
        osg::Camera* createCamera( int x, int y, int w, int h,
                                   osgQt::GLWidget *QTObject,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        void rotateCamera(double theta,double phi,double rad);
        void setImageDirectories();
        void LoadTextures();
        void setTexture(QString ID, int texIndex);
        virtual void paintEvent( QPaintEvent* event ) { frame(); }
        bool getPrimaryUser();
        osg::Vec3 mapWallPos(osg::Vec3 pt);
        void updateDockables();
        void updateCursors();
        static void set_z_position(OSGObjectBase* p, int z);
        void updateCursor(PlanarObject* cursor, vct3 kinect_hand_position, osg::Group* group);

        void setupSkeletons();
        void readConfigFile();

        // Tooltips //
        void loadToolTips();
        void setTooltip(QString key, int id);
        void hideTooltip(int id);

        void toggleStacks(){};
        void loadIndex();

        void loadLarge(osg::Vec3 v);
        void loadCollection(QString collection, int start);
        void showCollection(QString collection, int start);

        void loadLargeFromTurner(turn_t side);
        void showPageCollection(QString c, QString id, int start);
        void loadPageTurner(osg::Vec3 v);
        void loadTurn(turn_t turn);
        void showPageTurner(QString collection, QString id);
        QStringList* loadManuscriptCollection(QString collection, int start);
        void showManuscriptCollection(QString collection, int start);

        void updateEnvironment();
        void handleUserInput();

        void findImage(osg::Vec3 pt, QString& col, int& index, QString& id);
        int findNearestIndex(osg::Vec3 pt);
        
        // Camera Control Variables //
        osg::ref_ptr<osg::Camera> _camera;
        ServoCamera _camServo;
        osg::Vec3 _cameraOrbit;
        osg::Vec3 _cameraStart;

        QTimer _selectTimer;
        QTimer _backTimer;
        QTimer _turnTimer;
        QTimer _relaxTimer;
        // QTimer _leaveTimer;
        QTimer _matrixTimer;
        QTimer _arrayTimer;

        // USERS //
        int numActiveUsers;

    private:
        //String Formatter
        static QString labelFormatter(QString, int);
        static void thumbnailCalc(double& x, double& y);
        static void numThumbnailCalc(int& start, int& end, int size);
        static void largeCalc(double& x, double& y);
        static void turnCalc(double& x, double& y);
        void setTitle(QString t, QString st, int pO, int pT);
        void updateButtons(int tStart, int tTotal);

        //Thumbnail Measurements. TODO: Potentialy make struct.
        const static int THUMBNAIL_SIDE= 90;
        const static int THUMB_OFFSET_X=-772;
        const static int THUMB_OFFSET_Y=210; 
        const static int THUMB_GAP_X= 170;
        const static int THUMB_GAP_Y=-138;
        const static int THUMB_TEXT_SIZE=5;
        const static int THUMB_TEXT_OFFSET=-THUMBNAIL_SIDE/2;
        const static int LARGE_SIDE= 720;
        const static int TURNER_SIDE= 500;
        const static int TURNER_OFFSET= -20;

        const static int HIGHLIGHT_ZOOM=100;

        const static int LABEL_LENGTH= 30;

        const static int DISPLAY_WIDTH=10;
        const static int DISPLAY_HEIGHT=4;
        const static int NUM_THUMBNAILS=DISPLAY_HEIGHT*DISPLAY_WIDTH;

        const static int CLICK_THRESHOLD=220;
        const static int SELECTION_DELAY=750;

        const static int VIEWPORT_DEFAULT_ZOOM = 400;

        //TODO: These maybe should be passed inside the methods instead of being global
        QList<PicFlyerApp::Model::DisplayImage *>* collectionImages;
        QList<PicFlyerApp::Model::Opening *>* dispOpenings;


        vct2 mapEnvPos(vct3 in);

	int col_start;
	int pages_start;

    //Why is all of this public?
    public:

        vct3 WKSP_OFFSET_PIC;

        // Icons //
        ObjectMap _actableIcons;
        ObjectMap _dockableIcons;
        SphereMap _cursorIcons;
        PlaneMap _planarCursors;
        PlaneMap _page_arrows;


        // Wrappers //
        OSGObjectBase* _actableWrapper;
        OSGObjectBase* _dockableWrapper;  
        OSGObjectBase* _envWrapper;
        OSGObjectBase* _imageWrapper;
        OSGObjectBase* _indexWrapper;
        OSGObjectBase* _skelWrapper;
        OSGObjectBase* _hudWrapper;
        OSGObjectBase* _cursorWrapper;
        OSGObjectBase* _cameraAttachedObjects;
        OSGObjectBase* _largeImageWrapper;
        OSGObjectBase* _openingWrapper;
        osg::ref_ptr<OSGObjectBase> _titleWrapper;

        // Images //
        PlaneList _imageThumbs;
        PlaneList _indexThumbs;
        
        // Skeletons //
        SkeletonPtrList _skels;   
                                   
        // OPENGL and QT //
        osgQt::GLWidget* glWidget;
        osg::ref_ptr<osg::Group> root;
        
        static bool _wallspace;
        static bool& isInWallspace(){return _wallspace;};

        // Variables
        QStringList assetPaths;

        QList<osg::ref_ptr< osg::TextureRectangle > > _assetTextures;
        QList<osg::ref_ptr< osg::TextureRectangle > > _imageTextures;
        QList<osg::ref_ptr< osg::TextureRectangle > > _indexTextures;
        // TextureList _assetTextures;
        // TextureList _imageTextures;
        // TextureList _indexTextures;

        QStringList _indexNames;
        QStringList _indexLabels;
        QStringList _imageLabels;

        //Title stuff
        osg::ref_ptr<osg::Geode> titleGeode;
        osg::ref_ptr<osgText::Text> title;
        osg::ref_ptr<osgText::Text> subtitle;
        osg::ref_ptr<osgText::Text> pageInd;

        // Collections
        //TODO: see notes on lines
        QStringList _collectionIDs; //Should probably be eliminated in a refactor
        QStringList _manuscriptIDs; //Should probably be eliminated in a refactor
        // QMap<QString, PicFlyerApp::Model::PictureCollection *>* _unorderedCollections;
        // QMap<QString, PicFlyerApp::Model::ManuscriptCollection *>* _manuscriptCollections;
        // PicFlyerApp::Model::PictureCollectionFactory* arbFac;

        //Default images
        static osg::ref_ptr<osg::Image> defaultCoverImage;
        static osg::ref_ptr<osg::Image> defaultPageImage;
        QList< osg::ref_ptr<osg::TextureRectangle> > _leftTex;
        QList< osg::ref_ptr<osg::TextureRectangle> > _rightTex;

        // Large Image 
        QList<osg::ref_ptr< osg::TextureRectangle > > _large;
        // TextureList _large;
        
        //TODO: ELIMINATE WHEN FIXED
        PlanarObject* _largeImage;
        PlanarObject* _pleaseWait;
        PlanarObject* _highlight;
        PlanarObject* _highlight_active;
        // TexturedPlane* _largeImage;
        // TexturedPlane* _pleaseWait;
        // TexturedPlane* _highlight;

        // Opening
        QList<osg::ref_ptr< osg::TextureRectangle > > _opening;
        // TextureList _opening;
        PlanarObject* _openingImage1;
        PlanarObject* _openingImage2;
        // TexturedPlane* _openingImage1;
        // TexturedPlane* _openingImage2;

        osg::Vec3 _imageLocation;

        QString assetDir;
        QString resourceDir;
        QString imageDir;

        // ToolTips //
        QString tooltipDir;
        QStringList tooltipPaths;
        QList<QPixmap*> toolTipImages;

        QLabel* tooltip1;
        QLabel* tooltip2;
        QLabel* tooltip3;
        QLabel* tooltip4;
        //////////////

        // ZOOM
        bool zoom_active;

        // Pan
        bool panning_active;

        int dataTextureIndexStart;
        int dataTextureIndex;

        int primaryUserID;
        int firstUserID;
        int secondUserID;
        bool useKinect;

        bool userInRange;
        QString panning_hand;

        static const double GUI_MAX_X =  1.0;
        static const double GUI_MIN_X = -1.0;
        static const double GUI_MAX_Y =  0.5;
        static const double GUI_MIN_Y = -0.5;
        
        vct2 WKSP_OFF;
        envState_t _envState;
        OSGObject* waitingObj;
        int runtime;
        bool paused;
        vct3 _torsoLast;
        osg::Vec3d _imageLast;
        vct2 _handLast;

        vct3 pr_saved;
        vct3 pl_saved;

        bool _snap;

        std::vector<int> button_call;

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
        void select();
        void back();
        void turn();
        // void leave();        
        // void setMatrixMode();
        // void setArrayMode();
        
    protected:
        // Timers //
        QTimer _timer;
        QTimer _dataTimer; 
        QTimer _delay;   
    };    
}
#endif

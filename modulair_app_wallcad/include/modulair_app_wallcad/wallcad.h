// GPL License
// no tabs, tab = 2 spaces
// class member  xxx_xxx_
// class method  CamelCase


#ifndef modulair_wallcad_h
#define modulair_wallcad_h

#include <modulair_core/modulair_app_base.h>
#include <modulair_core/modulair_app_base_qt.h>

#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>
#include <osgQt/GraphicsWindowQt>
#include <osgDB/ReadFile>
#include <osg/Camera>


#include <QtCore/QTimer>
#include <QtGui>


namespace modulair{

class WallCadApp;
//class KeyboardHandler : public osgGA::GUIEventHandler
//{
//public:
//  bool handle(const osgGA::GUIEventAdapter &,
//              osgGA::GUIActionAdapter &);
//  virtual void accept(osgGA::GUIEventHandlerVisitor& v);
//  virtual void setup(WallCadApp* app);
//  WallCadApp* appPtr;
//};


class WallCadApp : public QWidget, public ModulairAppBase, public osgViewer::Viewer
{
  Q_OBJECT

public:
  WallCadApp(std::string app_name, ros::NodeHandle nh, int event_deque_size);
  ~WallCadApp();

  // modulair appbase virtual methods
  bool build();
  bool start();
  bool stop();
  bool pause();
  bool resume();

  // osg qt functions
  QWidget* addViewWidget( osg::Camera* camera,
                          osg::Node* scene );

  osg::Camera* createCamera( int x, int y, int w, int h,
                             const std::string& name="",
                             bool windowDecoration=false );


public Q_SLOTS:
  void update();
  void updateApp();

protected:
  // timer
  QTimer timer_;
  QTimer data_timer_;

  QString asset_path_;
  bool paused_;

  // osg & qt
  osgQt::GLWidget* glwidget_;
  osg::ref_ptr<osg::Group> root_;

};


} // ns::modulair



#endif // modulair_wallcad_h

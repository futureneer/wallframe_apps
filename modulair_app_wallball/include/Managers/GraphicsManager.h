
#ifndef GRAPHICS_MANAGER
#define GRAPHICS_MANAGER

#include <irrlicht/irrlicht.h>
#include <string>
#include <QWidget>
#include <QGLWidget>

//using namespace irr;
//using namespace core;
//using namespace scene;
//using namespace video;
//using namespace io;
//using namespace gui;

class GraphicsManager {

public:

	static void setQWidget(QWidget* qWidget);

	static GraphicsManager* singleton();
	static void shutdown();

	bool tick();
    void resize(int w, int h);

    irr::scene::IMeshSceneNode* getMeshSceneNode(irr::io::path meshFilepath, irr::io::path textureFilepath);
	//void resizeGL(int width, int height, QGLWidget* widget);

    int* m_Scores;

private:

	static QWidget* s_QWidget;

	static GraphicsManager* s_Singleton;

	GraphicsManager();
	~GraphicsManager();
	
	// _____________________________ GRAPHICS MEMBERS
	
    irr::IrrlichtDevice* m_Device;
    irr::video::IVideoDriver* m_VideoDriver;
    irr::scene::ISceneManager* m_SceneManager;
    irr::gui::IGUIEnvironment* m_GUIEnvironment;
	
};

#endif // GRAPHICS_MANAGER

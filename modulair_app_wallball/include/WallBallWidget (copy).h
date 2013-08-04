#ifndef WALLBALLWIDGET_H
#define WALLBALLWIDGET_H

#include <QWidget>
#include <QGLWidget>
#include <QPaintEvent>
#include <QResizeEvent>

#include <WallBall.h>

//namespace lair {

class WallBallWidget : public QWidget
{
    Q_OBJECT;

public:
    WallBallWidget(QWidget *parent = 0);

    //    WallBallWidget(QWidget *parent = 0, QWidget *appManager = 0, QString appID = "null", bool useKin = false);

    ~WallBallWidget();
    void doResize(int w, int h);
    
    /*    public Q_SLOTS:
        void config();
        void pause();
        void unpause();
        void resume();
        void suspend();
        void pullData(); 
        void increment();
        void recieveDiscreteGesture(QMap<int,int> events){}*/

protected:
    void resizeEvent (QResizeEvent* event);
	void paintEvent ( QPaintEvent * event );
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);

private:
	static bool s_RunPhysics;
	static void* physicsThreadMethod(void* data);
}; // class WallBallWidget

//} // namespace lair

#endif // WALLBALLWIDGET_H

#ifndef WALLBALLWIDGET_H
#define WALLBALLWIDGET_H

#include <QWidget>
#include <QGLWidget>
#include <QPaintEvent>
#include <QResizeEvent>

#include <modulair_core/modulair_app_base_qt.h>

#include <WallBall.h>

//namespace lair {

class WallBallWidget : public modulair::ModulairAppBaseQt
{
    //Q_OBJECT;

public:
    WallBallWidget(std::string app_name, ros::NodeHandle nh, int event_deque_size);
    //WallBallWidget(QWidget *parent = 0);

    //    WallBallWidget(QWidget *parent = 0, QWidget *appManager = 0, QString appID = "null", bool useKin = false);

    ~WallBallWidget();
    void doResize(int w, int h);
    
    bool build();
    bool start();
    bool stop();
    bool pause();
    bool resume();

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

    // USERS //
    void updateUsers();

    int numActiveUsers;
    int images_per_user_;

    bool activeUsers[12];
    int joint_increments[12];
    bool prev_activeUsers[12];

    // Qt Events
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

#ifndef _osgobject_h
#define _osgobject_h
#include "OSGObjectWrapper.h"

#include <QtGui>
#include <QtCore/QTimer>

namespace lair{
class OSGObject : public QObject, public OSGObjectWrapper{
	Q_OBJECT

public:
	OSGObject(	QObject* parent,
				bool active,
				QList<osg::ref_ptr< osg::TextureRectangle > >* texPointer, 
				QString type);
	OSGObject();
	~OSGObject(){};
	bool hasInside(OSGObject* obj);
	void setEngagedTo(QString ID, OSGObject* obj);
	void setDisengaged();
	void setLabel(QString l);
	void activeScaling(double pos);
	QString engagedID;
	OSGObject* engagedObject;
    osg::Vec3 focusPoint;
    osg::Vec3 idlePoint;
protected:
private:
	QObject* myParent;
	bool isActive;
	QTimer _trigger;
	QTimer _idleTimer;
	QString _label;
Q_SIGNALS:
	void triggered(QString l);
	void triggered();
	void inUse();
public Q_SLOTS:
	void signalTriggered();
	void moveToFocus(int id);
	void waitAndMoveToIdle();
	void moveToIdle();
};
}
#endif
#ifndef _planarobject_h
#define _planarobject_h
// Includes //
#include "OSGObjectBase.h"
#include <osgText/Text>

namespace lair{

	class PlanarObject;
	typedef QList<PlanarObject*> PlaneList;
	typedef std::map<QString,PlanarObject*> PlaneMap;

	class PlanarObject : public OSGObjectBase
	{
	public:
		PlanarObject(	double xmin, double ymin, double zmin,
                     	double xmax, double ymax, double zmax,
        		     	QList<osg::ref_ptr< osg::TextureRectangle > >* tex, 
        		     	int texIndex, osg::Vec3 center);
    PlanarObject(	double xmin, double ymin, double zmin,
                     	double xmax, double ymax, double zmax,
		        		     	QList<osg::ref_ptr< osg::TextureRectangle > >* tex,
		        		     	osg::Vec3 center);
  	
		~PlanarObject();
		void initGeometry(int index);
		void applyNewTexture(int tindex);
		osg::ref_ptr<osg::Node> generatePlane(int index);
		void setTexture(int index);
		int getTexIndex();
		bool triggerBehavior(QString type);
		void setText(QString t);
		void setText(QString t, int sz, osg::Vec4 color, osg::Vec3 pos);
		void showText();
		void hideText();
		void setTransparency(double val);
		void setID(QString id);
		void setCollection(QString id);

		QString _id;
		QString _collection;
		int _texIndex;

	protected:
	private:
		QString _text;
		TextureList* _textures;
		osg::ref_ptr<osgText::Text> _textLabel;
		osg::ref_ptr<osg::Geode> _textGeode;
		osg::ref_ptr<osg::PositionAttitudeTransform> _textXform;
		bool _textVisible;
		osg::Vec3 _textScale;
		bool _init;

	};
}
#endif
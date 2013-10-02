#ifndef DISPLAYIMAGE_H
#define DISPLAYIMAGE_H

#include <QString>


#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include "ImageStoreDir.h"


namespace PicFlyerApp 
{
	namespace Model
	{
		class DisplayImage
		{
		public:
			DisplayImage(ImageStoreEntry);
			~DisplayImage();

			QString getName();

			osg::ref_ptr<osg::Image> getImage(QBool thumb=QBool(false));

		protected:
			// osg::ref_ptr<osg::Image> image;
			// osg::ref_ptr<osg::Image> thumb;
			ImageStoreEntry data;
		};
	}
}

#endif //DISPLAYIMAGE_H

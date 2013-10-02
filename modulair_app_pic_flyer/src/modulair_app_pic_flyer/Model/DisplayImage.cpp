#include <modulair_app_pic_flyer/Model/DisplayImage.h>
#include <QFile>
#include <iostream>


namespace PicFlyerApp
{
	namespace Model
	{
		DisplayImage::DisplayImage(ImageStoreEntry d)
		{
			this->data = d;
			// this->image = 0;
			// this->thumb = 0;
		}

		DisplayImage::~DisplayImage() {}

		QString DisplayImage::getName()
		{

			if (data.getTitle().isEmpty())
				return data.getImagePath();

			else
				return data.getTitle();
		}

		osg::ref_ptr<osg::Image> DisplayImage::getImage(QBool return_thumb) 
		{
			if (return_thumb) 
			{
				if (data.getThumbPath().isEmpty()) 
				{
						// Return full sized image instead.
						return getImage(QBool(false));
				} 
				else 
				{
						return osgDB::readImageFile(data.getThumbPath().toStdString());
				}
			}
			else 
			{

				return osgDB::readImageFile(data.getImagePath().toStdString());

			}
		}
	}
}

#ifndef PAGE_H
#define PAGE_H

#include <QString>

#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include "DisplayImage.h"
#include "ImageStoreDir.h"

namespace PicFlyerApp {

	namespace Model {

	class Page : public DisplayImage
	{
	public:
		Page(ImageStoreEntry, QString,QString);
		~Page();
		QString getManuscriptName();
		QString getPageNumbering();
		QBool hasImage(); //TODO: ask schiff - do we want to have this, or a default image?
		// QString getPThumbnailName(QString name);
		
		QString getName();

		void clearImages();

		enum Type
		{
			R,
			V,
			FRONT_MATTER_FLY_LEAF_R,
			FRONT_MATTER_FLY_LEAF_V,
			FRONT_MATTER_PASTE_DOWN,
			BINDING_FRONT_COVER,
			BINDING_SPINE,
			BINDING_BACK_COVER,
			END_MATTER_PASTE_DOWN,
			END_MATTER_FLY_LEAF_R,
			END_MATTER_FLY_LEAF_V,
			MISC
		};
		/**
	  Returns the ENUM type of this image, based ont eh filename.
	  @param QString The file.  From this we intuit the type of the image
	  */
		static Type getType(QString);

	private:
		QString pageNumbering; //eg 001r
		QString manuscriptName;
		QString realPageNumbering;
	};
	}
}
#endif // PAGE_H

#include <modulair_app_pic_flyer/Model/Page.h>
#include <QStringList>
#include <QFile>
#include <iostream>

using namespace osg;

namespace PicFlyerApp {

	namespace Model {

		Page::Page(ImageStoreEntry d, QString numbering, QString manuscriptName) : DisplayImage(d)
		{

			// this->imageFile=imageFile;
			this->pageNumbering = numbering;
			this->realPageNumbering = numbering;

			realPageNumbering.truncate(realPageNumbering.lastIndexOf("."));
			//realPageNumbering = new QString(realPageNumbering->mid(realPageNumbering->indexOf(".") + 1,realPageNumbering->length()).replace(".", " - "));

			this->manuscriptName=manuscriptName;
		}

		Page::~Page() {}

		/**
  Gets the string that represents where this page is.  Something like 120r (120th folio, recto side)
  */
		QString Page::getPageNumbering() {
			return realPageNumbering;
		}

		/**
  Gets the name of the manuscript this image comes from
  */
		QString Page::getManuscriptName() {
			return manuscriptName;
		}

		/**
  Returns whether or not this page contains the image which is the scan of this page.  returns false if we only have the default/missing image instead
  */
		QBool Page::hasImage() {
			if (!QFile(data.getImagePath()).exists()) {
				return QBool(false);
			}
			return QBool(true);
		}


		// QString Page::getThumbnailName(QString name) {
		// 	//std::cout << "Thumbnail of " << name.toStdString() << " is ";
		// 	name.insert(name.lastIndexOf('.'),".thumb");
		// 	//std::cout << name.toStdString() << std::endl;
		// 	return name;
		// }

		Page::Type Page::getType(QString fileName) {
			if (fileName.contains("binding.frontcover")) {
				return Page::BINDING_FRONT_COVER;
			} else if (fileName.contains("fromtmatter.pastedown")) {
				return Page::FRONT_MATTER_PASTE_DOWN;
			} else if (fileName.contains("frontmatmatter.flyleaf")) {
				//exampine page number
				if (fileName.split(".")[1].contains("r")) {
					return Page::FRONT_MATTER_FLY_LEAF_R;
				} else {
					return Page::FRONT_MATTER_FLY_LEAF_V;
				}
			} else if(fileName.contains("endmatter.flyleaf")) {
				//exampine page number
				if (fileName.split(".")[1].contains("r")) {
					return Page::END_MATTER_FLY_LEAF_R;
				} else {
					return Page::END_MATTER_FLY_LEAF_V;
				}
			} else if (fileName.contains("endmatter.pastedown")) {
				return Page::END_MATTER_PASTE_DOWN;
			} else if (fileName.contains("binding.backcover")) {
				return Page::BINDING_BACK_COVER;
			} else if (fileName.contains("binding.spine")) {
				return Page::BINDING_SPINE;
			} else if (fileName.split(".")[1].contains("r")) {
				return Page::R;
			} else if (fileName.split(".")[1].contains("v")) {
				return Page::V;
			}
			return Page::MISC;
		}

		QString Page::getName()
		{
			if(data.getTitle().isEmpty())
				return this->manuscriptName.append(this->pageNumbering.prepend(" "));
			else
				return data.getTitle();
		}


	}
}

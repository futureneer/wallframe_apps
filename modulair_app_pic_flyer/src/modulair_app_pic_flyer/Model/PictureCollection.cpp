#include <modulair_app_pic_flyer/Model/PictureCollection.h>
#include <QFile>

namespace PicFlyerApp {

	namespace  Model {

		PictureCollection::PictureCollection(ImageStoreDir* dir, QList<DisplayImage *> *collection)
		{
			this->directory=dir;

			name=directory->getTitle();
			// std::cerr<<"Name is: "<<name.toStdString()<<std::endl;

			description=directory->getDescription();

			//lets check and make sure things at least exist
			//Irrelevant for this structure
			// QString filename(ModelSettings::getInstance()->rootDirectory());
			// filename.append(this->commonName)
			// 		.append("/")
			// 		.append(this->commonName)
			// 		.append(".images.csv");

			//this->imagesFile=filename;

			this->collection=collection;
		}

		PictureCollection::~PictureCollection() {

			for (int i=0; i<collection->length(); i++) {
				delete(collection->at(i));
			}

			delete(collection);

			delete(directory);
		}


		QList<DisplayImage *> *PictureCollection::getImages() {
			return collection;
		}

		osg::ref_ptr<osg::Image> PictureCollection::getFirst(QBool thumb)
		{
			if(collection==NULL)
			{
				return NULL;
			}

			return collection->first()->getImage(thumb);
		}

		unsigned int PictureCollection::getPageCount() {
			if (collection!=NULL) {
				return collection->length();
			}
			return 0;
		}

		QString PictureCollection::getName() {
			return this->name;
		}

		QString PictureCollection::getDescription() {
			return this->getDescription();
		}

		QString PictureCollection::getPermission() {
			return directory->getPermission();
		}

	}
}

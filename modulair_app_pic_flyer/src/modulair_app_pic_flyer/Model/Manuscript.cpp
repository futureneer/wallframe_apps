#include <modulair_app_pic_flyer/Model/Manuscript.h>
#include <modulair_app_pic_flyer/Model/CSVParser.h>
#include <modulair_app_pic_flyer/Model/ModelSettings.h>

#include <iostream>
#include <QFile>

namespace PicFlyerApp {

	namespace Model {

		Manuscript::Manuscript(ImageStoreDir *dir, QString location, QString date, QString origin, QString repository, QString shelfMark, QList<Page *> *pages, QList<Opening *> *openings)
		{
			this->directory=dir;
			this->commonName=directory->getTitle();
			this->currentLocation=location;
			this->date=date;
			this->origin=origin;
			this->repository=repository;
			this->shelfMark=shelfMark;
			this->description=directory->getDescription();

			//lets check and make sure things at least exist
			/*QString filename(ModelSettings::getInstance()->rootDirectory());
			filename.append(this->commonName)
					.append("/")
					.append(this->commonName)
					.append(".images.csv");*/
			//std::cout << QDir::current().absolutePath().toStdString() << std::endl;
			//std::cout << filename.toStdString() << std::endl;
			//QString filename("/home/astalsi/computer-science-design-project/test-data/TEST1/TEST1.images.csv");
			//QFile inputFile(filename);
			//std::cout << QFileInfo(inputFile).absoluteFilePath().toStdString() << std::endl;

			//this->imagesFile=filename;

			this->pages=pages;
			this->openings=openings;
		}

		Manuscript::~Manuscript() {
			for (int i=0; i<openings->length(); i++) {
				delete(openings->at(i));
			}

			for (int i=0; i<pages->length(); i++) {
				delete(pages->at(i));
			}

			delete(openings);
			delete(pages);
			delete(directory);
		}

		osg::ref_ptr<osg::Image> Manuscript::getCover(QBool thumb)
		{
			if (pages==NULL)
				return NULL;
			for (int i=0; i<pages->length(); i++) {
				if (Page::getType(pages->at(i)->getPageNumbering())==Page::BINDING_FRONT_COVER) {
					return pages->at(i)->getImage(thumb);
				}
			}
			return NULL;
		}

		QList<Opening *> *Manuscript::getOpenings() {
			return openings;
		}

		unsigned int Manuscript::getOpeningCount() {
			if (openings!=NULL) {
				return openings->length();
			}
			return 0;
		}

		QList<Page *> *Manuscript::getPages() {
			return this->pages;
		}

		unsigned int Manuscript::getPageCount() {
			if (pages!=NULL) {
				return pages->length();
			}
			return 0;
		}

		QString Manuscript::getCommonName() {
			return this->commonName;
		}

		QString Manuscript::getCurrentLocation() {
			return this->currentLocation;
		}

		QString Manuscript::getDate() {
			return this->date;
		}

		QString Manuscript::getOrigin() {
			return this->origin;
		}

		QString Manuscript::getRepository() {
			return this->repository;
		}

		QString Manuscript::getShelfMark() {
			return this->shelfMark;
		}

		QString Manuscript::getName() {
		  return directory->getTitle();
		}

		QString Manuscript::getDescription()
		{
			return this->description;
		}

		QString Manuscript::getPermission() {
		    return directory->getPermission();
		}

		QList<DisplayImage *>* Manuscript::getImages()
		{
			//May leak memory. Check later
			QList<DisplayImage *>* temp;
			for(int x=0; x< pages->size(); x++)
			{
				temp->append(dynamic_cast<DisplayImage*>(pages->at(x)));
			}

			return temp;
		}
	}
}

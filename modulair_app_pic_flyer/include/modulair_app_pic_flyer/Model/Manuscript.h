#ifndef MANUSCRIPT_H
#define MANUSCRIPT_H

#include "IPictureCollection.h"
#include "Page.h"
#include "Opening.h"
#include "ImageStoreDir.h"
#include <QtCore>
#include <QList>

namespace PicFlyerApp {

    namespace Model {

        class Manuscript : public IPictureCollection
        {
            public:

        		/**
        		 * The objects pointed to by dir, pages, and openings are now owned by this instance.
        		 */
                Manuscript(ImageStoreDir *dir, QString location, QString date, QString origin, QString repository, QString shelfMark, QList<Page *> *pages, QList<Opening *> *openings);

                ~Manuscript();

                /**
                  Get list of all pages, in order
                  */
                QList<Page *> *getPages();

                //Get Images by spec
                QList<DisplayImage *>* getImages();

                /**
                  Get the number of pages in the list
                  */
                unsigned int getPageCount();

                /**
                  Get the set of all openings.
                  */
                QList<Opening *> *getOpenings();

                /**
                  Get the number of pages in the list
                  */
                unsigned int getOpeningCount();
                /**
                Returns an explicitly labeled front cover if one exists.  Otherwise returns null
                */
                osg::ref_ptr<osg::Image> getCover(QBool=QBool(false));

                QString getCommonName();
                QString getCurrentLocation();
                QString getDate();
                QString getOrigin();
                QString getRepository();
                QString getShelfMark();
                QString getDescription();
                QString getPermission();

                //Just return common name
                QString getName();

            protected:
                QString imagesFile;
                // ScannedImage* loadImage(QString,bool=false);
                QString getThumbnailName(QString);

                QString commonName;
                QString currentLocation;
                QString date;
                QString origin;
                QString repository;
                QString shelfMark;
                QString description;

                QList<Page *> *pages;
                QList<Opening *> *openings;
                ImageStoreDir *directory;
        };
    }
}

#endif // MANUSCRIPT_H

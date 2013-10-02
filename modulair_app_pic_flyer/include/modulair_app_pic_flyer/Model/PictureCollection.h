#ifndef PICTURECOLLECTION_H
#define PICTURECOLLECTION_H

#include "IPictureCollection.h"
#include "ImageStoreDir.h"
#include "DisplayImage.h"

namespace PicFlyerApp {

    namespace Model {

        class PictureCollection : public IPictureCollection
        {
            public:
        		/**
        		 * Pointer arguments now owned by PictureCollection.
        		 */
                PictureCollection(ImageStoreDir* dir, QList<DisplayImage *> *collection);

                ~PictureCollection();

                /**
                  Get list of all pages, in order
                  */
                QList<DisplayImage *>* getImages();

                /**
                    Get the first image (like a front cover)
                    */
                osg::ref_ptr<osg::Image> getFirst(QBool=QBool(false));
                /**
                  Get the number of pages in the list
                  */
                unsigned int getPageCount();

                QString getDescription();

                QString getPermission();

                QString getName();


            protected:
                //QString imagesFile;
                // ScannedImage* loadImage(QString,bool=false);
                QString getThumbnailName(QString);

                QString name;
                QString description;

                QList<DisplayImage *>* collection;
                ImageStoreDir* directory;
        };
    }
}


#endif

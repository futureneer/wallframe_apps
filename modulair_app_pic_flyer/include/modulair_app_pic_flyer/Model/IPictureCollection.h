#ifndef IPICTURECOLLECTION_H
#define IPICTURECOLLECTION_H

#include <QList>
#include "DisplayImage.h"

namespace PicFlyerApp {

    namespace Model {

        class IPictureCollection
        {
        public:
            virtual QList<DisplayImage *>* getImages()=0;
            virtual unsigned int getPageCount()=0;
            virtual QString getName()=0;
        };
    }
}

#endif // IPICTURECOLLECTION_H

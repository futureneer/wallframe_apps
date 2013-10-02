#ifndef PICTURECOLLECTIONFACTORY_H
#define PICTURECOLLECTIONFACTORY_H

#include <QString>
#include <QIODevice>
#include <QHash>
#include <QVector>
#include "PictureCollection.h"
#include "ImageStoreDir.h"

namespace PicFlyerApp {

    namespace Model {

        class PictureCollectionFactory
        {
        public:
            static PictureCollectionFactory *getInstance();
            PictureCollection* getCollectionByName(QString);
            QStringList* getCollectionNames();
            void setRoot(QString r);
            void setCollectionNames(QStringList n);
            ~PictureCollectionFactory();

        private:
            static QString root;
            static QStringList* names;
            PictureCollectionFactory();
            static PictureCollectionFactory *m_instance;
            //static QIODevice *bookscsv;
            QList<DisplayImage *> *getImages(ImageStoreDir* d, QString name);
        };

    }}

#endif // PICTURECOLLECTIONFACTORY_H

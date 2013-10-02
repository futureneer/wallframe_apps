
#include <modulair_app_pic_flyer/Model/PictureCollectionFactory.h>
#include <QtCore>
#include <iostream>

namespace PicFlyerApp {

    namespace Model {

        PictureCollectionFactory *PictureCollectionFactory::m_instance=NULL;
        QStringList* PictureCollectionFactory::names= new QStringList();
        QString PictureCollectionFactory::root=NULL;

        PictureCollectionFactory::PictureCollectionFactory()
        {
            m_instance=this;

            //This needs to grab the collections from the config file

        }

        PictureCollectionFactory *PictureCollectionFactory::getInstance() {
            if (m_instance==NULL) {
                m_instance= new PictureCollectionFactory;
            }
            return m_instance;
        }

        void PictureCollectionFactory::setRoot(QString r)
        {
            root = r;
        }

        void PictureCollectionFactory::setCollectionNames(QStringList n)
        {
            names = new QStringList(n);
        }

        QStringList* PictureCollectionFactory::getCollectionNames() {
            
            //Will be handled in constructor for  collections

            // if (names == NULL) {
            //     names = new QList<QString>;
            //     bookscsv->seek(0); //QFile Docs say to use seek in preference to reset() because of buffering
            //     CSVParser input(bookscsv);
            //     std::vector<QString> thisItem;
            //     while(input.getNextItem(&thisItem)) {
            //         names->push_back(thisItem[0]);
            //     }
            //}
            return names;
        }

        PictureCollection* PictureCollectionFactory::getCollectionByName(QString name) {
            
            if(!names->contains(name))
                return NULL;

            //Look for collection in filesystem, load it

            //if(!name.existsinfilesystem)
                //return NULL;

            //Build images
            QString directoryName(root + QString("/") + name);
            ImageStoreDir* dir= new ImageStoreDir(QDir(directoryName));    
            QList<DisplayImage *> *imageset = getImages(dir, name);

            return new PictureCollection(dir, imageset);
        }


        QList<DisplayImage *> *PictureCollectionFactory::getImages(ImageStoreDir* dir, QString name)
        {
            //Pull directory from config file
            

            QList<DisplayImage *>* imageset = new QList<DisplayImage *>;

            //Roll through directory, pickup pictures
            // std::cerr<<directoryName.toStdString()<<std::endl;
            // QDir imageDir(directoryName);
            // QStringList imageFiles = imageDir.entryList(QDir::Files | QDir::Readable, QDir::Name);

            // for(int x=0; x<imageFiles.size(); x++)
            // {
            //     imageset->append(new DisplayImage(directoryName+QString("/")+imageFiles.at(x)));
            // }

            QVector<ImageStoreEntry> entries = dir->readEntries();

            if(entries.size()==0)
            {
                std::cerr<<"No entries found for this collection"<<std::endl;
                exit(0);
            }

            for (int x=0; x<entries.size(); x++)
            {
                imageset->append(new DisplayImage((entries[x])));
            }

            return imageset;
        }

        PictureCollectionFactory::~PictureCollectionFactory() {
            delete names;
        }
    }
}

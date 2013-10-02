#include <modulair_app_pic_flyer/Model/ManuscriptCollectionFactory.h>
#include <QtCore>
#include <iostream>
#include <modulair_app_pic_flyer/Model/CSVParser.h>

namespace PicFlyerApp {

    namespace Model {

    	QHash<QString, ManuscriptCollection *>* ManuscriptCollectionFactory::manuscriptCollections= new QHash<QString, ManuscriptCollection *>();
    	QString ManuscriptCollectionFactory::root = NULL;
    	QStringList* ManuscriptCollectionFactory::names = NULL;

    	ManuscriptCollection* ManuscriptCollectionFactory::getCollectionByName(QString key)
    	{
    		if (!manuscriptCollections->contains(key))
    		{   			
    			manuscriptCollections->insert(key, new ManuscriptCollection(root + QString("/") + key));
    		}

    		return manuscriptCollections->value(key);
    	}

        void ManuscriptCollectionFactory::setRoot(QString r)
        {
            root=r;
        }

        void ManuscriptCollectionFactory::setCollectionNames(QStringList n)
        {
            names= new QStringList(n);
        }

    }
}

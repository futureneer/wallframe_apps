#include <modulair_app_pic_flyer/Model/CollectionStore.h>

#include <iostream>
#include <QtCore>

namespace PicFlyerApp
{
		namespace Model
		{
			QStringList CollectionStore::pictureCollectionIDs= QStringList();
			QStringList CollectionStore::manuscriptIDs= QStringList();
			QString CollectionStore::directoryRoot = QString("");
			QMap<QString, PicFlyerApp::Model::collectionType_t>* CollectionStore::allCollectionIDs= new QMap<QString, PicFlyerApp::Model::collectionType_t>();
			QMap<QString, PicFlyerApp::Model::PictureCollection *>* CollectionStore::unorderedCollections= new QMap<QString, PicFlyerApp::Model::PictureCollection *>();
			QMap<QString, PicFlyerApp::Model::ManuscriptCollection *>* CollectionStore::manuscriptCollections= new QMap<QString, PicFlyerApp::Model::ManuscriptCollection *>();
			PicFlyerApp::Model::PictureCollectionFactory* CollectionStore::arbFac= PicFlyerApp::Model::PictureCollectionFactory::getInstance();

			// QMap<QString, PicFlyerApp::Model::PictureCollection *>* CollectionStore::getPictureCollections()
			// {
			// 	return unorderedCollections;
			// }

			// QMap<QString, PicFlyerApp::Model::ManuscriptCollection *>* CollectionStore::getManuscripts()
			// {
			// 	return manuscriptCollections;
			// }

	        PictureCollection* CollectionStore::getPictureCollectionByID(QString id)
	        {
	        	if(pictureCollectionIDs.contains(id))
	        	{
	        		if(!unorderedCollections->contains(id))
	        		{
	        			unorderedCollections->insert(id, arbFac->getCollectionByName(id));
	        		}

        			return unorderedCollections->value(id);
	        	}

	        	return NULL;
	        }

	    	ManuscriptCollection* CollectionStore::getManuscriptCollectionByID(QString id)
	    	{
	    		if(manuscriptIDs.contains(id))
	    		{
	    			if(!manuscriptCollections->contains(id))
	    			{
	    				manuscriptCollections->insert(id, ManuscriptCollectionFactory::getCollectionByName(id));
	    			}

	    			return manuscriptCollections->value(id);
	    		}

	    		return NULL;

	    	}


	        PicFlyerApp::Model::collectionType_t CollectionStore::getCollectionType(QString id)
	        {
	        	return allCollectionIDs->value(id);
	        }

	        void CollectionStore::setPictureCollectionIDs(QStringList acn)
	        {
	        	pictureCollectionIDs=QStringList(acn);
	        	arbFac->setCollectionNames(acn);
	        	for(int i; i<pictureCollectionIDs.size(); i++)
	        	{
	        		allCollectionIDs->insert(pictureCollectionIDs.at(i), UNORDERED_COLLECTION);
	        	}
	        }

	        void CollectionStore::setManuscriptIDs(QStringList mn)
	        {
	        	manuscriptIDs=QStringList(mn);
	        	ManuscriptCollectionFactory::setCollectionNames(mn);
	        	for(int i; i<manuscriptIDs.size(); i++)
	        	{
	        		allCollectionIDs->insert(manuscriptIDs.at(i), MANUSCRIPT_COLLECTION);
	        	}
	        }
	        
			QStringList CollectionStore::getManuscriptIDs()
			{
				return manuscriptIDs;
			}

		    QStringList CollectionStore::getPictureCollectionIDs()
		    {
		    	return pictureCollectionIDs;
		    }

		    void CollectionStore::setDirectoryRoot(QString r)
		    {
		    	//TODO: Check if this works
		    	directoryRoot=r;
		    	arbFac->setRoot(r);
		    	ManuscriptCollectionFactory::setRoot(r);
		    }
		}

}

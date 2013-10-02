#ifndef COLLECTIONSTORE_H
#define COLLECTIONSTORE_H

#include <QString>
#include <QMap>
#include "ManuscriptCollection.h"
#include "ManuscriptCollectionFactory.h"
#include "PictureCollection.h"
#include "PictureCollectionFactory.h"

namespace PicFlyerApp
{

	namespace Model
	{
    	enum collectionType_t {	UNORDERED_COLLECTION,
    							MANUSCRIPT_COLLECTION};

		class CollectionStore
		{
		private:

			static QStringList pictureCollectionIDs; //Should probably be eliminated in a refactor
	        static QStringList manuscriptIDs; //Should probably be eliminated in a refactor
	        static QMap<QString, PicFlyerApp::Model::collectionType_t>* allCollectionIDs;
	        static QMap<QString, PicFlyerApp::Model::PictureCollection *>* unorderedCollections;
	        static QMap<QString, PicFlyerApp::Model::ManuscriptCollection *>* manuscriptCollections;
	        static PicFlyerApp::Model::PictureCollectionFactory* arbFac;
	        static QString directoryRoot;

	    public:

	    	// static QMap<QString, PicFlyerApp::Model::PictureCollection *>* getPictureCollections();
	        // static QMap<QString, PicFlyerApp::Model::ManuscriptCollection *>* getManuscripts();
	        static PictureCollection* getPictureCollectionByID(QString id);
	        static ManuscriptCollection* getManuscriptCollectionByID(QString id);
	        static PicFlyerApp::Model::collectionType_t getCollectionType(QString id);
	        static void setPictureCollectionIDs(QStringList acn);
	        static void setManuscriptIDs(QStringList mn);
	        static QStringList getManuscriptIDs();
	        static QStringList getPictureCollectionIDs();
	        static void setDirectoryRoot(QString r);


	    };
	}

}

#endif //COLLECTIONSTORE_H
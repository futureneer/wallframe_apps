#ifndef MANUSCRIPTCOLLECTIONFACTORY_H
#define MANUSCRIPTCOLLECTIONFACTORY_H

#include <QString>
#include <QIODevice>
#include <QHash>
#include "Manuscript.h"
#include "ManuscriptCollection.h"

namespace PicFlyerApp {

    namespace Model {

//Consider changing to namespace
class ManuscriptCollectionFactory
{
public:
    static ManuscriptCollection* getCollectionByName(QString);
    //~ManuscriptCollectionFactory();

    static void setRoot(QString r);
    static void setCollectionNames(QStringList n);
    
private:
    static QString root;
    static QStringList* names;
    static QHash<QString, ManuscriptCollection *>* manuscriptCollections;
};

    }}

#endif // MANUSCRIPTCOLLECTIONFACTORY_H

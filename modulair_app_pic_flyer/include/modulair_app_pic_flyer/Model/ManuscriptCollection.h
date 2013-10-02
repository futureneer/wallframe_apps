#ifndef MANUSCRIPTCOLLECTION_H
#define MANUSCRIPTCOLLECTION_H

#include <QString>
#include <QIODevice>
#include <QHash>
#include <QVector>
#include "Manuscript.h"
#include "ImageStoreDir.h"

namespace PicFlyerApp {

    namespace Model {

class ManuscriptCollection
{
public:
	ManuscriptCollection(QString);

	~ManuscriptCollection();

    Manuscript *getManuscriptByName(QString);

    QStringList *getManuscriptNames();

    osg::ref_ptr<osg::Image> genCover();
    
    QString getTitle();

private:
    QHash<QString, Manuscript *>* manuscripts;
    QStringList* names;
    QString root;
    QHash<QString, QStringList> books_table;

    QList<Opening *> *getOpenings(QList<Page *> *pages);

    QList<Page *> *getPages(ImageStoreDir* dir, QString name);
};

    }}

#endif // MANUSCRIPTCOLLECTION_H

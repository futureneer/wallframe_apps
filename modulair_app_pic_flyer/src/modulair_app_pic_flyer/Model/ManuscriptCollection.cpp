
#include <modulair_app_pic_flyer/Model/ManuscriptCollection.h>

#include <QtCore>
#include <QDir>

#include <iostream>

#include <modulair_app_pic_flyer/Model/CSVParser.h>

namespace PicFlyerApp {

    namespace Model {

        ManuscriptCollection::ManuscriptCollection(QString rt)
        {
            root = rt;
	        names = 0;
            manuscripts = new QHash<QString, Manuscript *>();

            QString books_csv_path = root + QString("/books.csv");
            QFile books_csv_file(books_csv_path);

            if (!books_csv_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            	std::cerr << "Could not open books.csv: " << books_csv_path.toStdString() << std::endl;
            }

            QTextStream books_csv_in(&books_csv_file);
            books_csv_in.setCodec("UTF-8");
            CSVParser books_csv_parser(&books_csv_in);

            while (books_csv_parser.hasNext()) {
                QStringList row = books_csv_parser.nextRow();
                books_table.insert(row[0], row);
            }
        }

        osg::ref_ptr<osg::Image> ManuscriptCollection::genCover()
        {
            if(manuscripts->isEmpty())
            {
                if (!names) {
                    getManuscriptNames();
                }

                getManuscriptByName(names->first());
            }
            // std::cerr<<"First Manuscript: "<<names->first().toStdString()<<std::endl;
            
            return manuscripts->value(manuscripts->keys().first())->getCover(QBool(true));
        }

        QStringList *ManuscriptCollection::getManuscriptNames() {
            if (!names) {
                names = new QStringList();
                QDir root_dir(root);
                QList<QString> keys = books_table.keys();

                for (int i = 0; i < keys.length(); i++) {
                    QString name = keys[i];

                    if (root_dir.exists(name)) {
                        names->push_back(name);
                        std::cerr << "NAME: " << name.toStdString() << std::endl;
                    } else {
                        std::cerr << "WARNING: MS doesn't exist: " << name.toStdString() << std::endl;
                    }
                }
            }

            return names;
        }

        Manuscript *ManuscriptCollection::getManuscriptByName(QString name) {
            if(!manuscripts->contains(name) && books_table.contains(name)) {
                QStringList row = books_table.value(name);
                ImageStoreDir *dir = new ImageStoreDir(QDir(root + QString("/") + name));

                QList<Page *>* pages = getPages(dir, row[0]);
                QList<Opening *> *openings = 0;

                if (pages) {
                    openings = getOpenings(pages);
                }

                manuscripts->insert(name, new Manuscript(dir, row[4], row[5], row[7], row[1], row[3], pages,openings));
            }


            return manuscripts->value(name);
        }

        QList<Opening *> *ManuscriptCollection::getOpenings(QList<Page *> *pages) {

            QList<Page *> *thisOpeningPages=NULL;
            QList<Opening *> *result = new QList<Opening *>;
            //while (pagesCSV.getNextItem(&thisPageResult)) {
            for (int i=0; i<pages->length(); i++) {

                //if we dont have a list of pages, lets start bulding one.
                if (!thisOpeningPages) {
                    thisOpeningPages = new QList<Page *>;
                }

                // std::cout << pages->at(i)->getPageNumbering().toStdString() << std::endl;

                switch(Page::getType(pages->at(i)->getPageNumbering())) {
                case Page::BINDING_BACK_COVER:
                case Page::BINDING_FRONT_COVER:
                case Page::BINDING_SPINE:
                case Page::MISC:
                    if (thisOpeningPages->length()>0) {
                        result->append(new Opening(thisOpeningPages));
                        thisOpeningPages = new QList<Page *>;
                    }
                    thisOpeningPages->append(pages->at(i));
                    result->append(new Opening(thisOpeningPages));
                    thisOpeningPages = new QList<Page *>;
                    break;
                default:
                    if (thisOpeningPages->length()>1) {
                        result->append(new Opening(thisOpeningPages));
                        thisOpeningPages = new QList<Page *>;
                    }
                    thisOpeningPages->append(pages->at(i));
                    break;
                }

            }

            if (thisOpeningPages->length()>0) {
                result->append(new Opening(thisOpeningPages));
            }

            return result;
        }

        QList<Page *> *ManuscriptCollection::getPages(ImageStoreDir* dir, QString name) {
        	QVector<ImageStoreEntry> entries = dir->readEntries();
        	QList<Page *> *res = new QList<Page *>;

        	for (int i = 0; i < entries.size(); i++) {
        		ImageStoreEntry entry = entries[i];
        		QString image_name = QFileInfo(entry.getImagePath()).fileName();

        		res->append(new Page(entry, image_name, name));
        	}

        	return res;
        }

        QString ManuscriptCollection::getTitle() {
            return ImageStoreDir(root).getTitle();
        }

        ManuscriptCollection::~ManuscriptCollection() {
            delete names;

            QHashIterator<QString, Manuscript *> i(*manuscripts);

             while (i.hasNext()) {
                 i.next();
                 delete i.value();
             }

             delete manuscripts;
        }
    }
}

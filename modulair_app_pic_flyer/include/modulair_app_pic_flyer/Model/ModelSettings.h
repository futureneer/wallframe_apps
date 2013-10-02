#ifndef MODELSETTINGS_H
#define MODELSETTINGS_H

#include <QString>
#include <QtCore>

#define SETTINGS_FOLDER_NAME "balaur-browse"
#define SETTINGS_FILE_NAME "balaur_browse.conf"

namespace PicFlyerApp {

    namespace Model {
        class ModelSettings
{
public:
    static ModelSettings* getInstance();
    QString booksCSVLocation();
    QString rootDirectory();
    QString imagesCSVSuffix();
private:
    ModelSettings();
    static ModelSettings* m_instance;
    static QString rootDir;
    static QString booksCSV;
    static QString imagesCSVSuf;
    //static const QString SETTINGS_BOOKS_CSV_LOCATION;

};

    }}

#endif // MODELSETTINGS_H

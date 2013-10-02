#include <modulair_app_pic_flyer/Model/ModelSettings.h>
#include <QDesktopServices>
#include <QFile>
#include <iostream>

namespace PicFlyerApp {

    namespace Model {
        ModelSettings* ModelSettings::m_instance = NULL;
        QString ModelSettings::rootDir = NULL;
        QString ModelSettings::booksCSV = NULL;
        QString ModelSettings::imagesCSVSuf = QString(".images.csv");

        //QString ModelSettings::SETTINGS_BOOKS_CSV_LOCATION = "../test-data/books.csv"

        ModelSettings* ModelSettings::getInstance() {
            if (m_instance == NULL) {
                m_instance = new ModelSettings();
            }
            return m_instance;
        }

        QString ModelSettings::booksCSVLocation() {
            return rootDirectory() + this->booksCSV;
        }

        QString ModelSettings::rootDirectory() {
            return this->rootDir;
        }

        QString ModelSettings::imagesCSVSuffix() {
            return this->imagesCSVSuf;
        }

    ModelSettings::ModelSettings()
    {

        //Grab from the picflier config file. See next block as well
        QFile settings(QDesktopServices::storageLocation(QDesktopServices::DataLocation) +
                       QString("/") + QString(SETTINGS_FOLDER_NAME) + QString("/") +
                       QString(SETTINGS_FILE_NAME));

        if (!settings.exists()) {
            std::cout << "Failed to find config! Your config goes in: ";
            std::cout << QDesktopServices::storageLocation(QDesktopServices::DataLocation).toStdString();
            std::cout << "/" << SETTINGS_FOLDER_NAME << "/" << SETTINGS_FILE_NAME << std::endl;
            throw new QtConcurrent::Exception();
        }

        if (!settings.open(QIODevice::ReadOnly | QIODevice::Text)) {
            std::cout << "Unable to open config file!" << std::endl;
            throw new QtConcurrent::Exception();
        }

        //for now only have very specific way of setting up this file
        this->rootDir=settings.readLine().trimmed() + QString("/");
        this->booksCSV=settings.readLine().trimmed();
        this->imagesCSVSuffix()=settings.readLine().trimmed();

        settings.close();
    }

}}

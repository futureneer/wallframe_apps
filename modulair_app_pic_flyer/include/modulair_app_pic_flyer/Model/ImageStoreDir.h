#ifndef IMAGE_STORE_DIR_H_
#define IMAGE_STORE_DIR_H_

#include <QDir>
#include <QVector>

namespace PicFlyerApp {
    namespace Model {

/**
 * An image in a ImageStoreDir.
 */
class ImageStoreEntry {
private:
	QString image_path;
	QString thumb_path;
	QString title;
	QString url;

public:
	ImageStoreEntry(QString image_path = QString(), QString thumb_path =
			QString(), QString title = QString(), QString url = QString());

	/**
	 * Return the title of the image.
	 * Empty if not available.
	 */
	QString getTitle() const;

	/**
	 * Return a url pointing to information about the image.
	 * Empty if not available.
	 */
	QString getUrl() const;

	/**
	 * Return path to the image file.
	 */
	QString getImagePath() const;

	/**
	 * Return path to the thumb. Empty if the thumb doesn't exist.
	 */
	QString getThumbPath() const;
};

/*
 * Class for managing a collection of images stored in a directory with metadata and thumbnails.
 * The thumbnails are stored in a thumbs subdirectory. The metadata is stored in a file collection.xml.
 * See https://wiki.library.jhu.edu/x/goNsAQ for more information.
 */
class ImageStoreDir {
private:
	QDir dir;
	QString title;
	QString description;
	QString permission;
	QString url;
	QString thumb_suffix;

public:
	ImageStoreDir(QDir dir);

	~ImageStoreDir();

	/**
	 * Return the directory storing the images.
	 */
	QDir getDir() const;

	/**
	 * A short description of the image collection.
	 * Empty if not available.
	 */
	QString getDescription() const;

    /**
     * A statement giving permission for use of images in the collection.
     * Empty if not available.
     */
    QString getPermission() const;

	/**
	 * The title of the image collection.
	 * Empty if not available.
	 */
	QString getTitle() const;

	/**
	 * A URL pointing to more information about the collection.
	 * Empty if not available.
	 */
	QString getUrl() const;

	/**
	 * Return a list of images available in the collection.
	 */
	QVector<ImageStoreEntry> readEntries();
};

    }}
#endif

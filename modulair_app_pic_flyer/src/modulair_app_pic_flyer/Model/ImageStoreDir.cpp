#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <iostream>

#include <modulair_app_pic_flyer/Model/ImageStoreDir.h>

XERCES_CPP_NAMESPACE_USE

using namespace std;

namespace PicFlyerApp {
    namespace Model {

// TODO: Use SAX or pull parser
// TODO: throw exceptions or log in standard way

static const char *COLLECTION_FILE_NAME = "collection.xml";
static const char *THUMB_DIR_NAME = "thumbs";

static QString get_child_element_value(DOMElement *el, const char *name) {
	QString result;

	XMLCh* xml_str_name = XMLString::transcode(name);

	for (el = el->getFirstElementChild(); el; el = el->getNextElementSibling()) {
		if (XMLString::equals(el->getNodeName(), xml_str_name)) {
			const XMLCh* xml_str_value = el->getTextContent();
			char *value = XMLString::transcode(xml_str_value);
			result = QString(value);
			XMLString::release(&value);
			break;
		}
	}

	XMLString::release(&xml_str_name);

	return result;
}

static QString get_attribute(DOMElement *el, const char *name) {
	XMLCh* xml_str_name = XMLString::transcode(name);
	const XMLCh* xml_str_value = el->getAttribute(xml_str_name);
	XMLString::release(&xml_str_name);

	char *value = XMLString::transcode(xml_str_value);
	QString result = QString(value);
	XMLString::release(&value);

	return result;
}

static XercesDOMParser* parse_file(QString xmlfile) {
	if (!QFile(xmlfile).exists()) {
		return 0;
	}

	XercesDOMParser* parser = new XercesDOMParser();

	ErrorHandler* errHandler = (ErrorHandler*) new HandlerBase();
	parser->setErrorHandler(errHandler);

	try {
		parser->parse(xmlfile.toStdString().c_str());
	} catch (const XMLException& e) {
		char* message = XMLString::transcode(e.getMessage());
		cerr << "ERROR: Exception message is: \n" << message << "\n";
		XMLString::release(&message);
		delete parser;
		return 0;
	} catch (const DOMException& e) {
		char* message = XMLString::transcode(e.getMessage());
		cerr << "ERROR: Exception message is: \n" << message << "\n";
		XMLString::release(&message);
		delete parser;
		return 0;
	}

	delete errHandler;

	return parser;
}

ImageStoreDir::ImageStoreDir(QDir dir) {
	this->dir = dir;

	QDir thumb_dir = QDir(dir.filePath(THUMB_DIR_NAME));

	// Peek at a thumb to determine the thumb extension

	if (thumb_dir.exists()) {
		QFileInfoList infolist = thumb_dir.entryInfoList();

		while (!infolist.isEmpty() && thumb_suffix.isEmpty()) {
			thumb_suffix =  infolist.takeFirst().suffix();
		}
	}

	try {
		XMLPlatformUtils::Initialize();
	} catch (const XMLException& e) {
		char* message = XMLString::transcode(e.getMessage());
		cerr << "Error during xerces-c init :" << endl << message << endl;
		XMLString::release(&message);
	}

	if (dir.exists(COLLECTION_FILE_NAME)) {
		XercesDOMParser*parser = parse_file(dir.filePath(COLLECTION_FILE_NAME));

		if (parser) {
			DOMDocument *doc = parser->getDocument();
			DOMElement *root = doc->getDocumentElement();

			this->title = get_child_element_value(root, "title");
			this->description = get_child_element_value(root, "description");
			this->url = get_child_element_value(root, "url");
			this->permission = get_child_element_value(root, "permission");

			delete parser;
		}
	} else {
		cerr << "ERROR: Could not find " << COLLECTION_FILE_NAME << " in "
				<< dir.path().toStdString() << endl;
	}
}

QVector<ImageStoreEntry> ImageStoreDir::readEntries() {
	XercesDOMParser*parser = parse_file(dir.filePath(COLLECTION_FILE_NAME));

	QVector<ImageStoreEntry> result = QVector<ImageStoreEntry>();

	if (!parser) {
		return result;
	}

	QDir thumb_dir = QDir(dir.filePath(THUMB_DIR_NAME));

	DOMDocument *doc = parser->getDocument();
	DOMElement *root = doc->getDocumentElement();

	// Iterate over all image elements
	// Do not use getElementsByTagName because it is very slow

	XMLCh* images_name = XMLString::transcode("images");
	XMLCh* image_name = XMLString::transcode("image");

	for (DOMElement *el1 = root->getFirstElementChild(); el1; el1 = el1->getNextElementSibling()) {
		if (!XMLString::equals(el1->getNodeName(), images_name)) {
			continue;
		}

		for (DOMElement *el2 = el1->getFirstElementChild(); el2; el2 = el2->getNextElementSibling()) {
			if (!XMLString::equals(el2->getNodeName(), image_name)) {
				continue;
			}

			QString filename = get_attribute(el2, "filename");

			if (filename.isEmpty()) {
				continue;
			}

			if (!dir.exists(filename)) {
				cerr << "WARNING: No such image " << qPrintable(filename) << endl;

				continue;
			}

			QString image_path = dir.filePath(filename);
			QString thumb_name = QFileInfo(image_path).completeBaseName().append('.').append(
					thumb_suffix);

			QString thumb_path;

			if (thumb_dir.exists(thumb_name)) {
				thumb_path = thumb_dir.filePath(thumb_name);
			} else {
				thumb_path = QString();
			}

			QString title = get_child_element_value(el2, "title");
			QString url = get_child_element_value(el2, "url");

			result.append(ImageStoreEntry(image_path, thumb_path, title, url));
		}
	}

	XMLString::release(&image_name);
	XMLString::release(&images_name);

	delete parser;

	return result;
}

QString ImageStoreDir::getDescription() const {
	return description;
}

QString ImageStoreDir::getPermission() const {
    return permission;
}

QString ImageStoreDir::getTitle() const {
	return title;
}

QString ImageStoreDir::getUrl() const {
	return url;
}

QDir ImageStoreDir::getDir() const {
	return dir;
}

ImageStoreDir::~ImageStoreDir() {
	try {
		XMLPlatformUtils::Terminate();
	} catch (xercesc::XMLException& e) {
		char* message = xercesc::XMLString::transcode(e.getMessage());
		cerr << "ERROR: xerces-c teardown: " << message << endl;
		XMLString::release(&message);
	}
}

ImageStoreEntry::ImageStoreEntry(QString image_path, QString thumb_path,
		QString title, QString url) {
	this->image_path = image_path;
	this->thumb_path = thumb_path;
	this->title = title;
	this->url = url;
}

QString ImageStoreEntry::getTitle() const {
	return title;
}

QString ImageStoreEntry::getImagePath() const {
	return image_path;
}

QString ImageStoreEntry::getThumbPath() const {
	return thumb_path;
}

QString ImageStoreEntry::getUrl() const {
	return url;
}

    }}

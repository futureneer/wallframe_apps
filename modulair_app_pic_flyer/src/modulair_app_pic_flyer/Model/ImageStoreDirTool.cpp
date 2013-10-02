/* Tool to interact with image store directories.
 */

#include <iostream>

#include "Model/ImageStoreDir.h"

using namespace std;
using namespace PicFlyerApp::Model;

int main(int argc, char *argv[]) {
	// For now just print out info

	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " image_dir" << endl;
		return 1;
	}

	ImageStoreDir store = ImageStoreDir(QDir(argv[1]));

	cout << "Store: " << argv[1] << endl;
	cout << "Title: " << qPrintable(store.getTitle()) << endl;
	cout << "Description: " << qPrintable(store.getDescription()) << endl;
	cout << "URL: " << qPrintable(store.getUrl()) << endl;

	QVector<ImageStoreEntry> entries = store.readEntries();

	cout << "Num images: " << entries.size() << endl;

	for (int i = 0; i < entries.size(); i++) {
		cout << endl;

		ImageStoreEntry entry = entries[i];

		cout << "  Image: " << qPrintable(entry.getImagePath()) << endl;
		cout << "  Thumb: " << qPrintable(entry.getThumbPath()) << endl;
		cout << "  Title: " << qPrintable(entry.getTitle()) << endl;
		cout << "  URL: " << qPrintable(entry.getUrl()) << endl;
	}

	return 0;
}

#ifndef CSVPARSER_H
#define CSVPARSER_H

#include <QtCore>

namespace PicFlyerApp {

	namespace Model {
		class CSVParser {
			public:
				CSVParser(QTextStream *input);

				bool hasNext();

				QStringList nextRow();

			private:
				QTextStream *input;
		};
	}
}

#endif // CSVPARSER_H

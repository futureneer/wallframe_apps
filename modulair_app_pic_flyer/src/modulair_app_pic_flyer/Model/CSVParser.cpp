#include <modulair_app_pic_flyer/Model/CSVParser.h>
#include <QStringList>
#include <iostream>

namespace PicFlyerApp {

	namespace Model {

		CSVParser::CSVParser(QTextStream *input)
		{
			this->input = input;
		}

		bool CSVParser::hasNext() {
			return !input->atEnd();
		}

		QStringList CSVParser::nextRow()
		{
			QStringList result;

			// TODO: Much simpler ways to do this!!

			QString line = input->readLine();
			QStringList lineParts = line.split(',');
			QString thisPartResult="";
			bool inQuotes=false;

			for (int i=0;
				 i<lineParts.size();
				 i++) {
				QString thisPart = lineParts[i];
				QString workingPart = thisPart.trimmed();

				if (workingPart.size()==0) {
					continue;
				}

				if (!inQuotes && workingPart.at(0)==QChar('"')) {
					inQuotes=true;
					//attach everything except the quote at the start
					workingPart=workingPart.right(workingPart.size()-1);
				}

				if (inQuotes && workingPart.endsWith('"')) {
					inQuotes=false;
					workingPart.truncate(workingPart.size()-1);
				} else if (inQuotes) {
					workingPart.append(", ");
				}

				thisPartResult.append(workingPart);

				if (!inQuotes) {
					result.push_back(thisPartResult.replace("\"\"","\""));
					thisPartResult="";
				}
			}

			return result;
		}
	}
}

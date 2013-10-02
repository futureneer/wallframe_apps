#ifndef OPENING_H
#define OPENING_H

#include <QList>
#include "Page.h"

namespace PicFlyerApp {

    namespace Model {

        class Opening
        {
            public:
                Opening(QList<Page *> *);
                ~Opening();
                int getLength();
                QList<PicFlyerApp::Model::Page *> *getPages();
                Page *getPageByIndex(int);
                QString getText();
            private:
                QList<Page *> *pages; //contians pages that should be in this opening, includeing null if a page is missing
        };
    }
}

#endif // OPENING_H

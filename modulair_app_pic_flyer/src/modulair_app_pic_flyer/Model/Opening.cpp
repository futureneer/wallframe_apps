#include <modulair_app_pic_flyer/Model/Opening.h>

namespace PicFlyerApp {

    namespace Model {

        Opening::Opening(QList<Page *> *pages)
        {
            this->pages=pages;
        }

        Opening::~Opening() {
            delete(pages);
        }

        int Opening::getLength() {
            return pages->length();
        }

        QList<Page *> *Opening::getPages() {
            return pages;
        }

        Page *Opening::getPageByIndex(int index) {
            return pages->at(index);
        }

        QString Opening::getText() {
            QString text;

            for (int i = 0; i < pages->length(); i++) {
                Page* page = pages->at(i);

                text += page->getName();

                if (i < pages->length() - 1) {
                    text += ", ";
                }
            }

            return text;
        }
    }
}

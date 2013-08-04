#include <WallBallWidget.h>

#include <QApplication>
#include <QWidget>
#include <QBoxLayout>

#include <iostream>

int main(int argc, char** argv) {
	
	QApplication a(argc, argv);

    WallBallWidget widget;

    widget.show();
    widget.move(1680,20);
    widget.setMinimumSize(5760,3240);

    a.exec();
}

#include <QApplication>

#include "TSWidget.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	TSWidget w;
	w.resize(600,400);
	w.show();

	return app.exec();
}

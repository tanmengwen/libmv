#include <QApplication>

#include "../MultiImageViewer.h"

int main(int argc, char *argv[])
{	
	QApplication app(argc, argv);

	MultiImageViewer w;
	w.resize(600,400);
	w.show();
	
	return app.exec();
}

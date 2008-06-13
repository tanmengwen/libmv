#include <QtGui>

#include "TwoViewWidget.h"
#include "Window.h"

Window::Window()
{
	twoViewWidget = new TwoViewWidget;

	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addWidget(twoViewWidget);
	setLayout(mainLayout);

	setWindowTitle(tr("Hello GL"));
}
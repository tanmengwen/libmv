#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include "TwoViewWidget.h"

class Window : public QWidget
{
	Q_OBJECT

	public:
		Window();

	private:
		TwoViewWidget *twoViewWidget;
};

#endif

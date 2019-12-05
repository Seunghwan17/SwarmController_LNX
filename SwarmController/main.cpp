#include "SwarmController.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	SwarmController w;
	w.show();
	return a.exec();
}

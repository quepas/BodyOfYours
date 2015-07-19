#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication app(argc,argv);
	app.setOrganizationName("Startup");
	app.setApplicationName("BodyOfYours");

	MainWindow main_window;
	main_window.show();

	return app.exec();
}

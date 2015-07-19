#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <RecFusion.h>

#include <QtWidgets/QMainWindow>

class QLabel;

/** \brief	Main window of QtReconstruction application
*/
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow();
	~MainWindow();

public slots:
	void onStartReconstruction();
	void onStopReconstruction();

	void processFrame();

private:
	QLabel* m_imgLabel;
	QLabel* m_recLabel;
	RecFusion::Sensor* m_sensor;
	RecFusion::Reconstruction* m_rec;
	RecFusion::ColorImage* m_imgColor;
	RecFusion::ColorImage* m_imgScene;
	RecFusion::DepthImage* m_imgDepth;

	bool m_reconstruct;
};


#endif

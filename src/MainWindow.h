#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <RecFusion.h>

#include <QtWidgets/QMainWindow>

class QLabel;
class QMessageBox;
class QTimer;

/** \brief	Main window of MultiViewReconstruction application
*/
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow();
	~MainWindow();

public slots:
	void processFrames();

	void calibrate();
	void performCalibration();
	void saveCalibration();
	void loadCalibration();

	void startReconstruction();
	void stopReconstruction();

private:
	QLabel* m_imgLabel[2];
	QLabel* m_recLabel[2];
	QMessageBox* m_calibMessageBox;
	QTimer* m_timer;

	RecFusion::Sensor* m_sensor[2];
	
	RecFusion::ColorImage* m_colorImg[2];
	RecFusion::DepthImage* m_depthImg[2];
	RecFusion::ColorImage* m_sceneImg[2];
	RecFusion::ColorImage* m_calibImgColor[2];
	RecFusion::DepthImage* m_calibImgDepth[2];

	RecFusion::Mat3 m_K[2];
	RecFusion::Mat4 m_sensorT[2];
	bool m_calibImgValid[2];

	bool m_reconstruct;
	bool m_calibrate;

	RecFusion::Reconstruction* m_rec;
};

#endif

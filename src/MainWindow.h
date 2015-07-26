#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <RecFusion.h>

#include <QtWidgets/QMainWindow>
#include <vector>

#include "SensorData.h"

class QLabel;
class QMessageBox;
class QTimer;

/** \brief  Main window of MultiViewReconstruction application
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
    QLabel* m_imgLabel[3];
    QLabel* m_recLabel[3];
    QMessageBox* m_calibMessageBox;
    QTimer* m_timer;

    RecFusion::Sensor* m_sensor[3];
    
    RecFusion::ColorImage* m_colorImg[3];
    RecFusion::DepthImage* m_depthImg[3];
    RecFusion::ColorImage* m_sceneImg[3];
    RecFusion::ColorImage* m_calibImgColor[3];
    RecFusion::DepthImage* m_calibImgDepth[3];

    RecFusion::Mat3 m_K[3];
    RecFusion::Mat4 m_sensorT[3];
    bool m_calibImgValid[3];

    bool m_reconstruct;
    bool m_calibrate;

    RecFusion::Reconstruction* m_rec;

  SensorData* sensor_data_;
  std::vector<SensorData*> sensors_data_;
  unsigned sensor_num_;
};

#endif

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <RecFusion.h>

#include <QtWidgets/QMainWindow>
#include <vector>

#include "Viewer.h"
#include "SensorData.h"
#include "PatientsWidget.h"

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

  void open3DModel();
  void onOpen3DModel(QString filename);

private:
  QLabel* m_imgLabel[3];
  QLabel* m_recLabel[3];
  QMessageBox* m_calibMessageBox;
  QTimer* m_timer;
  Viewer* viewer_;

  RecFusion::Sensor* m_sensor[3];
  RecFusion::Mat4 m_sensorT[3];

  bool m_reconstruct;
  bool m_calibrate;

  RecFusion::Reconstruction* m_rec;
  
  SensorData* sensor_data_;
  std::vector<SensorData*> sensors_data_;
  unsigned num_sensor_;

  QWindow* model_view_;
  PatientsWidget* patient_widget_;
};

#endif

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <RecFusion.h>

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QStackedLayout>
#include <vector>
#include <QGridLayout>

#include "Viewer.h"
#include "SensorData.h"
#include "PatientWidget.h"
#include "patientform.h"
#include "examinationform.h"
#include "Scanner.h"

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
  void calibrate();
  void performCalibration();
  void saveCalibration();
  void loadCalibration();

  void startReconstruction();
  void stopReconstruction();

  void openScan(QString filename);
  void calculateDiff();
  void calculateMirror();
  void onItemSelected(QTreeWidgetItem* current, QTreeWidgetItem* previous);

  void addPatient();
  void addExam();

private:
  QLabel* m_imgLabel[3];
  QLabel* m_recLabel[3];
  QMessageBox* m_calibMessageBox;
  Viewer* viewer_;
  Scanner* scanner_;
  QGridLayout* main_layout;
  QStackedLayout* stacked_layout_;
  PatientForm* patient_form_;
  ExaminationForm* exam_form_;

  RecFusion::Sensor* m_sensor[3];
  RecFusion::Mat4 m_sensorT[3];

  bool m_reconstruct;
  bool m_calibrate;

  RecFusion::Reconstruction* m_rec;
  
  SensorData* sensor_data_;
  std::vector<SensorData*> sensors_data_;
  unsigned num_sensor_;

  QWindow* model_view_;
  PatientWidget* patient_widget_;
};

#endif

#pragma once

#include <QAction>
#include <QLabel>
#include <QToolBar>

class ScannerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ScannerToolbar(QWidget* parent = nullptr);
  ~ScannerToolbar();

signals:
  void startReconstruction();
  void startCalibration();
  void stopReconstruction(QString meshFilePath);

public slots:
  void setEnabled(bool enabled);
  void showNumSensor(int numSensor);

private:
  QAction* start_recon_;
  QAction* stop_recon_;
  QAction* addExternalMesh_;
  QAction* start_calibration_;
  QLabel* numSensorLabel_;
};

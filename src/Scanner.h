#pragma once

#include <QObject>
#include <QImage>
#include <QTimer>
#include <QMessageBox>

#include <RecFusion.h>

#include "ImageData.h"
#include "SensorData.h"

class Scanner : public QObject
{
  Q_OBJECT
public:
  Scanner(QWidget* parent);
  ~Scanner();

  int numSensor();
  bool setLicense(QString file);
  bool hasLicense();

private:
  static const int MAX_NUM_SENSORS = 3;
  int numSensor_;
  RecFusion::Sensor* sensors_[MAX_NUM_SENSORS];
  RecFusion::Reconstruction* reconstruction_;
  SensorData* sensorsData_[MAX_NUM_SENSORS];
  bool recInProgress_;
  bool calibInProgress_;
  QTimer* timer_;
  QWidget* parent_;
  QMessageBox* m_calibMessageBox;

  bool hasLicense_;

public slots:
  void startReconstruction();
  void stopReconstruction();
  void calibrate();
  void saveCalibration();
  void loadCalibration();
  void performCalibration();
  void processFrames();

signals:
  void sendImages(QList<ImageData> image_rgb, QList<ImageData> image_recon);
};

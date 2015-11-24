#pragma once

#include "SensorData.h"
#include <QObject>
#include <QTimer>
#include <RecFusion.h>

class Scanner : public QObject
{
  Q_OBJECT
public:
  Scanner();
  ~Scanner();

private:
  static const int MAX_NUM_SENSORS = 3;
  int num_sensors_;
  RecFusion::Sensor* sensors_[MAX_NUM_SENSORS];
  RecFusion::Reconstruction* reconstruction_;
  SensorData* sensors_data_[MAX_NUM_SENSORS];
  bool rec_in_progress_;
  QTimer* timer_;

public slots:
  void startReconstruction();
  void stopReconstruction();
  void processFrames();

signals:
  void foundSensor(int num, QStringList names);
};

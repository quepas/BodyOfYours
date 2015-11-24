#pragma once

#include "SensorData.h"
#include <QObject>
#include <QImage>
#include <QTimer>
#include <RecFusion.h>

struct ImageData
{
  ImageData(QImage img, int w, int h)
    : image(img), width(w), height(h) {}
  QImage image;
  int width, height;
};

class Scanner : public QObject
{
  Q_OBJECT
public:
  Scanner();
  ~Scanner();

  int num_sensors() { return num_sensors_; }

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
  void sendImages(QList<ImageData> image_rgb, QList<ImageData> image_recon);
};

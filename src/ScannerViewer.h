#pragma once

#include "ImageData.h"
#include <QGridLayout>
#include <QLabel>
#include <QWidget>

class ScannerViewer : public QWidget
{
  Q_OBJECT
public:
  ScannerViewer(int numSensor, QWidget* parent = nullptr);
  ~ScannerViewer();

public slots:
  void displayImages(QList<ImageData> camera, QList<ImageData> scene);

private:
  static const int MAX_NUM_SENSORS = 3;
  QLabel* image_camera_[MAX_NUM_SENSORS];
  QLabel* image_scene_[MAX_NUM_SENSORS];
  QGridLayout* viewport_;
  int numSensor_;
};

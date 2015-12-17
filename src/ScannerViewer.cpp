#include "ScannerViewer.h"

#include <QDebug>

ScannerViewer::ScannerViewer(int numSensor, QWidget* parent /*= nullptr*/)
  : QWidget(parent),
    viewport_(new QGridLayout),
    numSensor_(numSensor)
{
  for (int i = 0; i < numSensor_; ++i) {
    image_camera_[i] = new QLabel;
    viewport_->addWidget(image_camera_[i], 0, i);
    image_scene_[i] = new QLabel;
    viewport_->addWidget(image_scene_[i], 1, i);
  }
  setLayout(viewport_);
}

ScannerViewer::~ScannerViewer()
{
  for (int i = 0; i < numSensor_; ++i) {
    delete image_camera_[i];
    delete image_scene_[i];
  }
  delete viewport_;
}

void ScannerViewer::displayImages(QList<ImageData> camera, QList<ImageData> scene)
{
  for (int i = 0; i < camera.size(); ++i) {
    image_camera_[i]->setPixmap(
      QPixmap::fromImage(camera[i].image).scaled(camera[i].width / 2.0,
      camera[i].height / 2.0, Qt::IgnoreAspectRatio,
      Qt::SmoothTransformation));
  }
  for (int i = 0; i < scene.size(); ++i) {
    image_scene_[i]->setPixmap(
      QPixmap::fromImage(scene[i].image).scaled(scene[i].width / 2.0,
      scene[i].height / 2.0, Qt::IgnoreAspectRatio,
      Qt::SmoothTransformation));
  }
}

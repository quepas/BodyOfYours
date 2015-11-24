#include "ScannerViewer.h"

#include <QDebug>

ScannerViewer::ScannerViewer(Scanner* scanner, QWidget* parent /*= nullptr*/)
  : viewport_(new QGridLayout), QWidget(parent)
{
  num_sensors_ = scanner->num_sensors();
  for (int i = 0; i < num_sensors_; ++i) {
    image_camera_[i] = new QLabel;
    viewport_->addWidget(image_camera_[i], 0, i);
    image_scene_[i] = new QLabel;
    viewport_->addWidget(image_scene_[i], 1, i);
  }
  connect(scanner, SIGNAL(sendImages(QList<ImageData>, QList<ImageData>)), this, SLOT(displayImages(QList<ImageData>, QList<ImageData>)));
  setLayout(viewport_);
}

ScannerViewer::~ScannerViewer()
{
  for (int i = 0; i < num_sensors_; ++i) {
    delete image_camera_[i];
    delete image_scene_[i];
  }
  delete viewport_;
}

void ScannerViewer::displayImages(QList<ImageData> camera, QList<ImageData> scene)
{
  for (int i = 0; i < camera.size(); ++i) {
    image_camera_[i]->setPixmap(QPixmap::fromImage(camera[i].image).scaled(camera[i].width / 2.0, camera[i].height / 2.0, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
  }
  for (int i = 0; i < scene.size(); ++i) {
    image_scene_[i]->setPixmap(QPixmap::fromImage(scene[i].image).scaled(scene[i].width / 2.0, scene[i].height / 2.0, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
  }
}

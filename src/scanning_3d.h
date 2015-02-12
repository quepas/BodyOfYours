#pragma once

#include "scanner_3d.h"

#include <QThread>

class Scanning3D : public QThread
{
  Q_OBJECT
public:
  Scanning3D(Scanner3D* scanner3d);
  void ScanVolumeData(bool scan_volume_data);
  void RestartReconstruction();
  void ReconstructAndSave(QString file_name);

protected:
  void run();

signals:
  void grabCameraFrame(FrameData* frame);
  void grabDepthFrame(FrameData* frame);
  void grabVolumeFrame(FrameData* frame);

private:
  Scanner3D* scanner_;
  bool scan_volume_data_;
};
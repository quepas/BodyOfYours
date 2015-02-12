#include "scanning_3d.h"

Scanning3D::Scanning3D(Scanner3D* scanner3d)
  : scanner_(scanner3d),
    scan_volume_data_(false)
{

}

void Scanning3D::run()
{
  bool is_running = true;
  while(is_running) {
    FrameData* frame = new FrameData();
    scanner_->GrabCameraFrame(frame);
    emit grabCameraFrame(frame);
    FrameData* depth_frame = new FrameData();
    scanner_->GrabDepthFrame(depth_frame);
    emit grabDepthFrame(depth_frame);
    if (scan_volume_data_) {
      FrameData* volume_frame = new FrameData();
      if (scanner_->GrabVolumeFrame(volume_frame)) {
        emit grabVolumeFrame(volume_frame);
      }
    }
    QThread::msleep(100);
  }
}

void Scanning3D::ScanVolumeData(bool scan_volume_data)
{
  scan_volume_data_ = scan_volume_data;
}

void Scanning3D::RestartReconstruction()
{
  scanner_->RestartReconstruction();
}

void Scanning3D::ReconstructAndSave(QString file_name)
{
  scanner_->ReconstructSurfaceToFile(file_name);
}

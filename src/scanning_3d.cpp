#include "scanning_3d.h"

Scanning3D::Scanning3D(Scanner3D* scanner3d, SensorViewer* viewer)
  : scanner_(scanner3d),
    viewer_(viewer)
{

}

void Scanning3D::run()
{
  bool is_running = true;
  while(is_running) {
    FrameData* frame = new FrameData();
    scanner_->GrabCameraFrame(frame);
    emit grabFrame(frame);
    QThread::msleep(100);
  }
}

#pragma once

#include <QStringList>

struct FrameData
{
  const void* data;
  int width, height, length;
};

class Scanner3D
{
public:
  Scanner3D() {}
  ~Scanner3D() {}

  virtual QStringList GetComputingDevices() = 0;
  virtual bool InitComputingDevice(int device_id) = 0;
  virtual bool GrabCameraFrame(FrameData* out_frame) = 0;
  virtual bool GrabDepthFrame(FrameData* out_frame) = 0;
private:
};

#pragma once

#include <QStringList>

class Scanner3D
{
public:
  Scanner3D() {}
  ~Scanner3D() {}

  virtual QStringList GetComputingDevices() = 0;
  virtual bool InitComputingDevice(int device_id) = 0;
private:
};

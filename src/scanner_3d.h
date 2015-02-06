#pragma once

#include <QStringList>

class Scanner3D
{
public:
  Scanner3D() {}
  ~Scanner3D() {}

  virtual QStringList GetCompatibleDevices() = 0;
  virtual bool InitCompatibleDevice(int device_id) = 0;
private:
};

#pragma once

#include <QStringList>

class Scanner3D
{
public:
  Scanner3D() {}
  ~Scanner3D() {}

  virtual QStringList GetCompatibleDevices() = 0;
private:
};

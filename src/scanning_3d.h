#pragma once

#include "scanner_3d.h"

#include <QThread>

class Scanning3D : public QThread
{
public:
  Scanning3D(Scanner3D* scanner3d);

protected:
  void run();

private:
  Scanner3D* scanner_;
};
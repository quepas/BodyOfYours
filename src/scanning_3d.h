#pragma once

#include "scanner_3d.h"

#include <QThread>

class Scanning3D : public QThread
{
  Q_OBJECT
public:
  Scanning3D(Scanner3D* scanner3d);

protected:
  void run();

signals:
  void grabFrame(FrameData* frame);

private:
  Scanner3D* scanner_;
};
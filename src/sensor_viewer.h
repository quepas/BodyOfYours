#pragma once

#include "scanner_3d.h"

#include <vtkImageViewer2.h>
#include <QVTKWidget.h>

class SensorViewer
{
public:
  SensorViewer(QVTKWidget* qvtk_widget);
  ~SensorViewer();

  void ShowFrame(FrameData* frame);

private:
  vtkImageViewer2* viewer_;
  QVTKWidget* qvtk_widget_;
  bool is_initialized;

};
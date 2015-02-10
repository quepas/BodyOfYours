#pragma once

#include "scanner_3d.h"

#include <vtkImageViewer.h>
#include <QVTKWidget.h>

class SensorViewer
{
public:
  SensorViewer(QVTKWidget* qvtk_widget);
  ~SensorViewer();

  void ShowFrame(FrameData* frame);

private:
  vtkImageViewer* viewer_;
  QVTKWidget* qvtk_widget_;

};
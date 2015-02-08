#pragma once

#include <vtkImageViewer.h>
#include <QVTKWidget.h>

class SensorViewer
{
public:
  SensorViewer(QVTKWidget* qvtk_widget);
  ~SensorViewer();

private:
  vtkImageViewer* viewer_;
  QVTKWidget* qvtk_widget_;

};
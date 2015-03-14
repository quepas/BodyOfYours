#pragma once

#include "scanner_3d.h"

#include <vtkImageViewer2.h>
#include <QVTKWidget.h>

class SensorViewer : public QVTKWidget
{
public:
  SensorViewer(QWidget* parent = nullptr);
  ~SensorViewer();

  void ShowFrame(FrameData* frame);
  void Clear();

private:
  vtkImageViewer2* viewer_;
  bool is_initialized;

};
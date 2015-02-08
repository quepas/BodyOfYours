#include "sensor_viewer.h"

#include <vtkRenderWindow.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>

SensorViewer::SensorViewer(QVTKWidget* qvtk_widget)
  : qvtk_widget_(qvtk_widget),
    viewer_(vtkImageViewer::New())
{
  vtkRenderWindow* render_window = viewer_->GetRenderWindow();
  qvtk_widget_->SetRenderWindow(render_window);
  viewer_->SetupInteractor(qvtk_widget_->GetInteractor());
  viewer_->SetColorWindow(233);
  qvtk_widget_->show();
}

SensorViewer::~SensorViewer()
{
  viewer_->Delete();
  qvtk_widget_->close();
}

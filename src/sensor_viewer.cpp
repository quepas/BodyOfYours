#include "sensor_viewer.h"

#include <vtkImageFlip.h>
#include <vtkImageImport.h>
#include <vtkImageViewer2.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>

SensorViewer::SensorViewer(QVTKWidget* qvtk_widget)
  : qvtk_widget_(qvtk_widget),
    viewer_(vtkImageViewer::New())
{
  vtkRenderWindow* render_window = viewer_->GetRenderWindow();
  qvtk_widget_->SetRenderWindow(render_window);
  viewer_->SetupInteractor(qvtk_widget_->GetInteractor());
  qvtk_widget_->show();
}

SensorViewer::~SensorViewer()
{
  viewer_->Delete();
  qvtk_widget_->close();
}

void SensorViewer::ShowFrame(FrameData* frame)
{
  vtkSmartPointer<vtkImageViewer2> imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  vtkSmartPointer<vtkImageImport> imageImport =vtkSmartPointer<vtkImageImport>::New();
  imageImport->SetWholeExtent(0, frame->width-1, 0, frame->height-1, 0, 0);
  imageImport->SetDataExtentToWholeExtent();
  imageImport->SetDataScalarTypeToUnsignedChar();
  imageImport->SetNumberOfScalarComponents(3);
  imageImport->SetImportVoidPointer((char*)(frame->data));
  imageImport->Update();

  vtkSmartPointer<vtkImageFlip> imageYFlip = vtkSmartPointer<vtkImageFlip>::New();
  imageYFlip->SetFilteredAxis(1);
  imageYFlip->SetInputConnection(imageImport->GetOutputPort());
  imageYFlip->Update();

  imageViewer->SetInputConnection(imageYFlip->GetOutputPort());
  qvtk_widget_->SetRenderWindow(imageViewer->GetRenderWindow());
  // disable mouse events
  qvtk_widget_->setEnabled(false);
  imageViewer->SetupInteractor(qvtk_widget_->GetInteractor());
  imageViewer->Render();
}

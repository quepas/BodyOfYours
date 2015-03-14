#include "sensor_viewer.h"

#include <vtkImageFlip.h>
#include <vtkImageImport.h>
#include <vtkImageViewer2.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>

SensorViewer::SensorViewer(QWidget* parent /*= nullptr*/)
  : QVTKWidget(parent),
    viewer_(vtkImageViewer2::New()),
    is_initialized(false)
{
  // disable mouse events
  setEnabled(false);
}

SensorViewer::~SensorViewer()
{
  viewer_->Delete();
  close();
}

void SensorViewer::ShowFrame(FrameData* frame)
{
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

  viewer_->SetInputConnection(imageYFlip->GetOutputPort());
  // fix for bug in vtk lib with non-optional connection needed when initializing image viewer
  if (!is_initialized) {
    SetRenderWindow(viewer_->GetRenderWindow());
    viewer_->SetupInteractor(GetInteractor());
    is_initialized = true;
  }
  viewer_->Render();
}

void SensorViewer::Clear()
{
  viewer_->GetRenderer()->Clear();
}

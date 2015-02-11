#include "scanningwindow.h"
#include "ui_scanningwindow.h"

ScanningWindow::ScanningWindow(Scanner3D* scanner, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanningWindow),
  scanning_(new Scanning3D(scanner))
{
  ui->setupUi(this);
  camera_viewer_ = new SensorViewer(ui->scansCameraViewer);
  depth_viewer_ = new SensorViewer(ui->scansDepthViewer);
  volume_viewer_ = new SensorViewer(ui->scansVolumeViewer);
  connect(scanning_, SIGNAL(grabCameraFrame(FrameData*)), this, SLOT(captureCameraFrame(FrameData*)));
  connect(scanning_, SIGNAL(grabDepthFrame(FrameData*)), this, SLOT(captureDepthFrame(FrameData*)));
  connect(scanning_, SIGNAL(grabVolumeFrame(FrameData*)), this, SLOT(captureVolumeFrame(FrameData*)));
}

ScanningWindow::~ScanningWindow()
{
  delete volume_viewer_;
  delete depth_viewer_;
  delete camera_viewer_;
  delete ui;
}

void ScanningWindow::on_startScanButton_clicked()
{
  if (!scanning_->isRunning()) {
    scanning_->start();
  }
}

void ScanningWindow::on_stopScanButton_clicked()
{

}

void ScanningWindow::captureCameraFrame(FrameData* frame)
{
  camera_viewer_->ShowFrame(frame);
  delete frame;
}

void ScanningWindow::captureDepthFrame(FrameData* frame)
{
  depth_viewer_->ShowFrame(frame);
  delete frame;
}

void ScanningWindow::captureVolumeFrame(FrameData* frame)
{
  volume_viewer_->ShowFrame(frame);
  delete frame;
}

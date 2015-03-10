#include "scanningwindow.h"
#include "ui_scanningwindow.h"

#include <QDateTime>

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
  scanning_->ScanVolumeData(true);
}

void ScanningWindow::on_stopScanButton_clicked()
{
  scanning_->ScanVolumeData(false);
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

void ScanningWindow::on_restartScanButton_clicked()
{
  scanning_->RestartReconstruction();
}

void ScanningWindow::on_saveScanButton_clicked()
{
  QString date_time = QDateTime::currentDateTime().toString("yyyy_MM_dd#HH_mm_ss");
  scanning_->ReconstructAndSave(
    "./data/patients/"
    + scanned_patient_.id()
    + "/scans/"
    + scanned_patient_.name() + "_" + scanned_patient_.surname()
    + "#" + date_time
    + ".ply");
}

void ScanningWindow::on_cancelScanButton_clicked()
{
  scanning_->terminate();
  this->close();
}

void ScanningWindow::StartGrabbingData()
{
  volume_viewer_->Clear();
  scanning_->start();
}

void ScanningWindow::Show(Patient scanned_patient)
{
  scanned_patient_ = scanned_patient;
  setWindowTitle("Scanning patient: " + scanned_patient.name() + " " + scanned_patient.surname());
  show();
}

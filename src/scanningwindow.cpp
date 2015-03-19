#include "scanningwindow.h"
#include "ui_scanningwindow.h"
#include "scaninfodialog.h"

#include <QDateTime>

ScanningWindow::ScanningWindow(Scanner3D* scanner, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanningWindow),
  scanning_(new Scanning3D(scanner))
{
  ui->setupUi(this);
  connect(scanning_, SIGNAL(grabCameraFrame(FrameData*)), this, SLOT(captureCameraFrame(FrameData*)));
  connect(scanning_, SIGNAL(grabDepthFrame(FrameData*)), this, SLOT(captureDepthFrame(FrameData*)));
  connect(scanning_, SIGNAL(grabVolumeFrame(FrameData*)), this, SLOT(captureVolumeFrame(FrameData*)));
}

ScanningWindow::~ScanningWindow()
{
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
  ui->scansCameraViewer->ShowFrame(frame);
  delete frame;
}

void ScanningWindow::captureDepthFrame(FrameData* frame)
{
  ui->scansDepthViewer->ShowFrame(frame);
  delete frame;
}

void ScanningWindow::captureVolumeFrame(FrameData* frame)
{
  ui->scansVolumeViewer->ShowFrame(frame);
  delete frame;
}

void ScanningWindow::on_restartScanButton_clicked()
{
  scanning_->RestartReconstruction();
}

void ScanningWindow::on_saveScanButton_clicked()
{
  ScanInfoDialog* scan_info_dialog_ = new ScanInfoDialog(scanning_, scanned_patient_);
  connect(scan_info_dialog_, SIGNAL(UpdatePatientSignal(Patient)), parent(), SLOT(UpdatePatientSlot(Patient)));
  scan_info_dialog_->setAttribute(Qt::WA_DeleteOnClose);
  scan_info_dialog_->show();
}

void ScanningWindow::on_cancelScanButton_clicked()
{
  scanning_->terminate();
  this->close();
}

void ScanningWindow::StartGrabbingData()
{
  ui->scansVolumeViewer->Clear();
  scanning_->start();
}

void ScanningWindow::Show(Patient scanned_patient)
{
  scanned_patient_ = scanned_patient;
  setWindowTitle("Scanning patient: " + scanned_patient.name() + " " + scanned_patient.surname());
  show();
}

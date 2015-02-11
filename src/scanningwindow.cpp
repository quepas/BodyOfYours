#include "scanningwindow.h"
#include "ui_scanningwindow.h"

ScanningWindow::ScanningWindow(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanningWindow)
{
  ui->setupUi(this);
  camera_viewer_ = new SensorViewer(ui->scansCameraViewer);
  depth_viewer_ = new SensorViewer(ui->scansDepthViewer);
  volume_viewer_ = new SensorViewer(ui->scansVolumeViewer);
}

ScanningWindow::~ScanningWindow()
{
  delete ui;
}

void ScanningWindow::on_startScanButton_clicked()
{

}

void ScanningWindow::on_stopScanButton_clicked()
{

}

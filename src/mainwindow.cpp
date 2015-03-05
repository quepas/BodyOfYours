#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scans_viewer.h"
#include "scans_tree.h"
#include "resources.h"
#include "scanning_3d.h"

#include <QFileSystemModel>
#include <QRegExp>
#include <QDebug>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <QVTKWidget.h>

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::io::loadPLYFile;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow),
    scanner3d_(new RemeScanner3D()),
    add_patient_dialog_(new AddPatientDialog)
{
  ui->setupUi(this);
  scanning_window_ = new ScanningWindow(scanner3d_),
  scans_viewer_ = new ScansViewer(ui->scansViewer);
  scans_tree_ = new ScansTree(ui->scansTree, Resources::SCANS_DATA_PATH);
  scans_data_tree_ = new ScansDataTree(ui->scansDataTree, scanning_window_);
  ui->computingDevicesComboBox->addItems(scanner3d_->GetComputingDevices());
  // set defualt icons
  for (int i = 0; i < ui->computingDevicesComboBox->maxVisibleItems(); ++i) {
    ui->computingDevicesComboBox->setItemIcon(i, QIcon(Resources::ICON_SYNC));
  }
  ui->scanButton->setIcon(QIcon(Resources::ICON_WARNING));
  ui->scanButton->setDisabled(true);
  connect(add_patient_dialog_, SIGNAL(AddPatientSignal(PatientData)), this, SLOT(AddPatientSlot(PatientData)));
  ui->addPatientButton->setIcon(QIcon(Resources::ICON_ADD));
  ui->removePatientButton->setIcon(QIcon(Resources::ICON_REMOVE));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_scansTree_doubleClicked(const QModelIndex &index)
{
  PointCloud<PointXYZ>::Ptr ply_cloud (new PointCloud<PointXYZ>);
  QString file_path = scans_tree_->model()->filePath(index);
  QRegExp ply_files_only("*.ply");
  ply_files_only.setPatternSyntax(QRegExp::Wildcard);
  if (ply_files_only.exactMatch(file_path)) {
    loadPLYFile(file_path.toStdString(), *ply_cloud);
    scans_viewer_->ShowPointCloud(ply_cloud);
  }
}

void MainWindow::on_computingDevicesComboBox_currentIndexChanged(int index)
{
  int device_id = index - 1;
  if (device_id > -1) {
    ui->computingDevicesComboBox->setItemData(0, "", Qt::UserRole-1);
    if (scanner3d_->InitComputingDevice(device_id)) {
      ui->computingDevicesComboBox->setItemIcon(index, QIcon(Resources::ICON_OK));
      ui->scanButton->setIcon(QIcon(Resources::ICON_OK));
      ui->scanButton->setDisabled(false);
      scans_data_tree_->SetScanActionEnable(true);
    } else {
      ui->computingDevicesComboBox->setItemIcon(index, QIcon(Resources::ICON_ERROR));
      ui->computingDevicesComboBox->setItemData(index, "", Qt::UserRole-1);
      ui->scanButton->setIcon(QIcon(Resources::ICON_WARNING));
      ui->scanButton->setDisabled(true);
      scans_data_tree_->SetScanActionEnable(false);
    }
  }
}

void MainWindow::on_scanButton_clicked()
{
  scanning_window_->show();
  scanning_window_->StartGrabbingData();
}

void MainWindow::on_addPatientButton_clicked()
{
  add_patient_dialog_->show();
}

void MainWindow::AddPatientSlot(PatientData data)
{
  scans_data_tree_->model()->AddPatientToTree(data);
  scans_data_tree_->model()->SavePatientToDisc(data);
  scans_data_tree_->model()->SavePatientMetadata(data);
}

void MainWindow::on_removePatientButton_clicked()
{
  scans_data_tree_->RemoveSelected();
}

void MainWindow::ModifyPatientSlot(PatientData data)
{

}

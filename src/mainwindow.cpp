#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scans_viewer.h"
#include "scans_tree.h"

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
    scanner3d_(new RemeScanner3D())
{
  ui->setupUi(this);
  scans_viewer_ = new ScansViewer(ui->scansViewer);
  scans_tree_ = new ScansTree(ui->scansTree, QDir::currentPath() + "/data");
  ui->combatibleDevicesComboBox->addItems(scanner3d_->GetCompatibleDevices());
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

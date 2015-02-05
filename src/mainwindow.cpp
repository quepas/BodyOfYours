#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scans_viewer.h"
#include "scans_tree.h"

#include <QFileSystemModel>
#include <QDebug>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <QVTKWidget.h>

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::io::loadPLYFile;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  scans_viewer_ = new ScansViewer(ui->scansViewer);
  scans_tree_ = new ScansTree(ui->scansTree, QDir::currentPath() + "/data");
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_scansTree_doubleClicked(const QModelIndex &index)
{
  PointCloud<PointXYZ>::Ptr ply_cloud (new PointCloud<PointXYZ>);
  loadPLYFile(scans_tree_->model()->fileName(index).toStdString(), *ply_cloud);
  scans_viewer_->ShowPointCloud(ply_cloud);
}

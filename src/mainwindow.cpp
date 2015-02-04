#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileSystemModel>
#include <QDebug>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <QVTKWidget.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

pcl::visualization::PCLVisualizer pviz ("PCL Visualizer", false);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ply_cloud (new pcl::PointCloud<pcl::PointXYZ>);
bool ply_loaded;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  SetupScansTree();
  ply_loaded = false;
  vtkSmartPointer<vtkRenderWindow> renderWindow = pviz.getRenderWindow();
  ui->scansViewer->SetRenderWindow (renderWindow);
  pviz.setupInteractor (ui->scansViewer->GetInteractor (), ui->scansViewer->GetRenderWindow ());
  ui->scansViewer->show();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::SetupScansTree()
{
  QFileSystemModel* model = new QFileSystemModel;
  QString data_root_path = QDir::currentPath() + "";
  model->setRootPath(data_root_path);
  QTreeView* tree = ui->scansTree;
  tree->setModel(model);
  tree->setRootIndex(model->index(data_root_path));
  tree->setColumnWidth(0, 150);
  tree->setColumnWidth(1, 50);
  tree->setColumnWidth(2, 70);
  tree->setColumnWidth(3, 100);
}


void MainWindow::on_scansTree_doubleClicked(const QModelIndex &index)
{
  QFileSystemModel* model = (QFileSystemModel*) ui->scansTree->model();
  pcl::io::loadPLYFile(model->fileName(index).toStdString(), *ply_cloud);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr ply_cloud2 (ply_cloud);
  if (ply_loaded) {
    pviz.updatePointCloud(ply_cloud2, "asd");
  } else {
     pviz.addPointCloud(ply_cloud2, "asd");
  }
  ply_loaded = true;
}

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileSystemModel>

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  SetupScansTree();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::SetupScansTree()
{
  QFileSystemModel* model = new QFileSystemModel;
  QString data_root_path = QDir::currentPath() + "/data/";
  model->setRootPath(data_root_path);
  QTreeView* tree = ui->scansTree;
  tree->setModel(model);
  tree->setRootIndex(model->index(data_root_path));
  tree->setColumnWidth(0, 150);
  tree->setColumnWidth(1, 50);
  tree->setColumnWidth(2, 70);
  tree->setColumnWidth(3, 100);
}

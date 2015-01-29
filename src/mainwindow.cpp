#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileSystemModel>

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_refreshScansTreeButton_clicked()
{
  QFileSystemModel* model = new QFileSystemModel;
  QString data_root_path = QDir::currentPath() + "/data/";
  model->setRootPath(data_root_path);
  QTreeView* tree = ui->scansTree;
  tree->setModel(model);
  tree->setRootIndex(model->index(data_root_path));
}

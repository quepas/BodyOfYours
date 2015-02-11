#include "scanningwindow.h"
#include "ui_scanningwindow.h"

ScanningWindow::ScanningWindow(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanningWindow)
{
  ui->setupUi(this);
}

ScanningWindow::~ScanningWindow()
{
  delete ui;
}

#include "scaninfodialog.h"
#include "ui_scaninfodialog.h"

ScanInfoDialog::ScanInfoDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog),
  scan_()
{
  ui->setupUi(this);
  setWindowTitle("Create scan");
}

ScanInfoDialog::ScanInfoDialog(Scan scan, QWidget *parent /*= 0*/) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog),
  scan_(scan)
{
  ui->setupUi(this);
  setWindowTitle("Update scan");
}

ScanInfoDialog::~ScanInfoDialog()
{
  delete ui;
}

void ScanInfoDialog::on_okButton_clicked()
{

}

void ScanInfoDialog::on_cancelButton_clicked()
{

}

#include "scaninfodialog.h"
#include "ui_scaninfodialog.h"

ScanInfoDialog::ScanInfoDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog)
{
  ui->setupUi(this);
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

#include "scaninfodialog.h"
#include "ui_scaninfodialog.h"

ScanInfoDialog::ScanInfoDialog(Patient owner, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog),
  scan_()
{
  ui->setupUi(this);
  setWindowTitle("Create scan");
  QDateTime now_time = QDateTime::currentDateTime();
  QString date_time = now_time.toString("yyyy_MM_dd#HH_mm_ss");
  QString filename = owner.name() + "_" + owner.surname() + "#" + date_time + ".ply";
  scan_.set_filename(filename);
  scan_.set_datetime(now_time);
}

ScanInfoDialog::ScanInfoDialog(Patient owner, Scan scan, QWidget *parent /*= 0*/) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog),
  scan_(scan)
{
  ui->setupUi(this);
  setWindowTitle("Update scan");
  ui->filenameText->setText(scan.filename());
  ui->scanDatetime->setDateTime(scan.datetime());
}

ScanInfoDialog::~ScanInfoDialog()
{
  delete ui;
}

void ScanInfoDialog::on_okButton_clicked()
{
  scanning_->ReconstructAndSave(
    "./data/patients/"
    + scanned_patient_.id()
    + "/scans/"
    + filename);
}

void ScanInfoDialog::on_cancelButton_clicked()
{
  ClearForm();
  close();
}

void ScanInfoDialog::ClearForm()
{
  ui->filenameText->clear();
  ui->scanNameText->clear();
  ui->scanDescriptionText->clear();
  ui->scanDatetime->clear();
}

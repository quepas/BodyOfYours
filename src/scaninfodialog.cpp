#include "scaninfodialog.h"
#include "ui_scaninfodialog.h"

ScanInfoDialog::ScanInfoDialog(Scanning3D* scanning, Patient owner, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog),
  scan_(),
  scanning_(scanning),
  patient_(owner)
{
  ui->setupUi(this);
  setWindowTitle("Create scan");
  QDateTime now_time = QDateTime::currentDateTime();
  QString date_time = now_time.toString("yyyy_MM_dd#HH_mm_ss");
  QString filename = owner.name() + "_" + owner.surname() + "#" + date_time + ".ply";
  scan_.set_filename(filename);
  scan_.set_datetime(now_time);
  ui->filenameText->setText(filename);
  ui->scanDatetime->setDateTime(now_time);
  ui->scanNameText->setText("Basic scan");
  ui->scanDescriptionText->setText("TO DO");
}

ScanInfoDialog::ScanInfoDialog(Scanning3D* scanning, Scan scan, QWidget *parent /*= 0*/) :
  QDialog(parent),
  ui(new Ui::ScanInfoDialog),
  scan_(scan),
  scanning_(scanning),
  patient_()
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
  // update scan data
  scan_.set_name(ui->scanNameText->text());
  scan_.set_description(ui->scanDescriptionText->toPlainText());
  scan_.set_filename(ui->filenameText->text());
  scan_.set_datetime(ui->scanDatetime->dateTime());
  // save scan
  QString filepath = "./data/patients/"
    + patient_.id()
    + "/scans/"
    + scan_.filename();
  scanning_->ReconstructAndSave(filepath);
  // emit changes
  QVector<Scan> scans = patient_.scans();
  scans.push_back(scan_);
  patient_.set_scans(scans);
  emit UpdatePatientSignal(patient_);
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

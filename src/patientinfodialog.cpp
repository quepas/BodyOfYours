#include "patientinfodialog.h"
#include "ui_patientinfodialog.h"

PatientInfoDialog::PatientInfoDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::PatientInfoDialog),
  only_update_(false)
{
  ui->setupUi(this);
  setWindowTitle("Create patient");
  ui->addPatientButton->setText("Create");
}

PatientInfoDialog::PatientInfoDialog(Patient patient, QWidget *parent /*= 0*/)
  : QDialog(parent),
    ui(new Ui::PatientInfoDialog),
    only_update_(true),
    updated_patient_(patient)
{
  ui->setupUi(this);
  setWindowTitle("Update patient");
  ui->nameText->setText(patient.name());
  ui->surnameText->setText(patient.surname());
  ui->additionalText->setText(patient.additional_info());
  ui->sexyComboBox->setCurrentIndex((patient.sex() ==  FEMALE) ? 0 : 1);
  ui->addPatientButton->setText("Update");
}

PatientInfoDialog::~PatientInfoDialog()
{
  delete ui;
}

void PatientInfoDialog::on_addPatientButton_clicked()
{
  Patient patient;
  patient.set_name(ui->nameText->text());
  patient.set_surname(ui->surnameText->text());
  patient.set_additional_info(ui->additionalText->toPlainText());
  bool is_female = ui->sexyComboBox->currentText() == "Female";
  patient.set_sex(is_female ? FEMALE : MALE);

  if (!patient.name().isEmpty()) {
    if (!only_update_) {
      emit CreatePatientSignal(patient);
    } else {
      patient.set_id(updated_patient_.id());
      emit UpdatePatientSignal(patient);
    }
    ClearData();
    close();
  }
}

void PatientInfoDialog::on_cancelAddPatientButton_clicked()
{
  ClearData();
  close();
}

void PatientInfoDialog::ClearData()
{
  ui->nameText->clear();
  ui->surnameText->clear();
  ui->additionalText->clear();
  ui->sexyComboBox->setCurrentIndex(0);
}

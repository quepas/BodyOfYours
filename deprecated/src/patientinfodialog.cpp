#include "patientinfodialog.h"
#include "ui_patientinfodialog.h"

PatientInfoDialog::PatientInfoDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::PatientInfoDialog),
  patient_()
{
  ui->setupUi(this);
  setWindowTitle("Create patient");
  ui->addPatientButton->setText("Create");
  patient_.set_id("none");
}

PatientInfoDialog::PatientInfoDialog(Patient patient, QWidget *parent /*= 0*/)
  : QDialog(parent),
    ui(new Ui::PatientInfoDialog),
    patient_(patient)
{
  ui->setupUi(this);
  setWindowTitle("Update patient");
  ui->addPatientButton->setText("Update");
  ui->nameText->setText(patient_.name());
  ui->surnameText->setText(patient_.surname());
  ui->additionalText->setText(patient_.additional_info());
  ui->sexyComboBox->setCurrentIndex((patient_.sex() ==  FEMALE) ? 0 : 1);
}

PatientInfoDialog::~PatientInfoDialog()
{
  delete ui;
}

void PatientInfoDialog::on_addPatientButton_clicked()
{
  patient_.set_name(ui->nameText->text());
  patient_.set_surname(ui->surnameText->text());
  patient_.set_additional_info(ui->additionalText->toPlainText());
  bool is_female = ui->sexyComboBox->currentText() == "Female";
  patient_.set_sex(is_female ? FEMALE : MALE);

  if (!patient_.name().isEmpty()) {
    if (patient_.id() == "none") {
      emit CreatePatientSignal(patient_);
    } else {
      emit UpdatePatientSignal(patient_);
    }
    ClearForm();
    close();
  }
}

void PatientInfoDialog::on_cancelAddPatientButton_clicked()
{
  ClearForm();
  close();
}

void PatientInfoDialog::ClearForm()
{
  ui->nameText->clear();
  ui->surnameText->clear();
  ui->additionalText->clear();
  ui->sexyComboBox->setCurrentIndex(0);
}

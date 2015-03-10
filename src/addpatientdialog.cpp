#include "addpatientdialog.h"
#include "ui_addpatientdialog.h"

AddPatientDialog::AddPatientDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AddPatientDialog),
  only_edit_(false)
{
  ui->setupUi(this);
  setWindowTitle("Add patient dialog");
}

AddPatientDialog::AddPatientDialog(Patient patient, QWidget *parent /*= 0*/)
  : QDialog(parent),
    ui(new Ui::AddPatientDialog),
    only_edit_(true)
{
  ui->setupUi(this);
  setWindowTitle("Add patient dialog");
  ui->nameText->setText(patient.name());
  ui->surnameText->setText(patient.surname());
  ui->additionalText->setText(patient.additional_info());
  ui->sexyComboBox->setCurrentIndex((patient.sex() ==  FEMALE_) ? 0 : 1);
}

AddPatientDialog::~AddPatientDialog()
{
  delete ui;
}

void AddPatientDialog::on_addPatientButton_clicked()
{
  Patient patient;
  patient.setName(ui->nameText->text());
  patient.setAdditionalInfo(ui->additionalText->toPlainText());
  bool is_female = ui->sexyComboBox->currentText() == "Female";
  patient.setSex(is_female ? FEMALE_ : MALE_);

  if (!patient.name().isEmpty()) {
    if (!only_edit_) {
      emit CreatePatientSignal(patient);
    } else {
    }
    ClearData();
    close();
  }
}

void AddPatientDialog::on_cancelAddPatientButton_clicked()
{
  ClearData();
  close();
}

void AddPatientDialog::ClearData()
{
  ui->nameText->clear();
  ui->additionalText->clear();
  ui->sexyComboBox->setCurrentIndex(0);
}

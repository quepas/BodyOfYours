#include "patientform.h"
#include "ui_patientform.h"

#include <QGroupBox>
#include <QDebug>

PatientForm::PatientForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PatientForm)
{
  ui->setupUi(this);
  form_buttons_ = new FormButtons(this);
  ui->formLayout->addWidget(form_buttons_->toQWidget());
  connect(form_buttons_, SIGNAL(buttonClicked(int)), this, SLOT(onButtonClicked(int)));
}

PatientForm::~PatientForm()
{
  delete ui;
}

void PatientForm::setData(const PatientData& data)
{
  ui->nameLineEdit->setText(data.name);
  ui->surnameLineEdit->setText(data.surname);
  ui->peselLineEdit->setText(data.pesel);
}

void PatientForm::clear()
{
  ui->nameLineEdit->clear();
  ui->surnameLineEdit->clear();
  ui->peselLineEdit->clear();
}

void PatientForm::setShowState(bool show_state)
{
  ui->formLayoutWidget->setDisabled(show_state);
}

void PatientForm::onButtonClicked(int button)
{
  switch (button) {
  case FormButtons::SAVE: {
    PatientData data;
    data.name = ui->nameLineEdit->text();
    data.surname = ui->surnameLineEdit->text();
    data.pesel = ui->peselLineEdit->text();
    emit savePatient(data); 
    break;
  }
  case FormButtons::CLEAR:
    clear();
    break;
  case FormButtons::CANCEL:
    break;
  case FormButtons::LOCK:
    break;
  case FormButtons::UNLOCK:
    break;
  default:
    break;
  }
  qDebug() << "PatientForm::onButtonClicked()";
}

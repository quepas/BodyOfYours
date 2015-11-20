#include "patientform.h"
#include "ui_patientform.h"

PatientForm::PatientForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PatientForm)
{
  ui->setupUi(this);
}

PatientForm::~PatientForm()
{
  delete ui;
}

void PatientForm::setData(QString name, QString surname)
{
  ui->lineEdit->setText(name);
  ui->lineEdit_2->setText(surname);
}

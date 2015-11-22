#include "patientform.h"
#include "ui_patientform.h"

#include <QDebug>

PatientForm::PatientForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PatientForm)
{
  ui->setupUi(this);
  save_button_ = new QPushButton(tr("Zapisz"));
  connect(save_button_, SIGNAL(clicked()), this, SLOT(onSave()));
  clear_button_ = new QPushButton(tr("Wyczysc"));
  connect(clear_button_, SIGNAL(clicked()), this, SLOT(onClear()));

  buttons_layout_ = new QHBoxLayout;
  //buttons_layout_->addStretch(1);
  buttons_layout_->addWidget(save_button_);
  buttons_layout_->addWidget(clear_button_);

  setLayout(buttons_layout_);
}

PatientForm::~PatientForm()
{
  delete save_button_;
  delete clear_button_;
  delete buttons_layout_;
  delete ui;
}

void PatientForm::setData(const PatientData& data)
{
  ui->nameLineEdit->setText(data.name);
  ui->surnameLineEdit->setText(data.surname);
  ui->peselLineEdit->setText(data.pesel);
}

void PatientForm::onSave()
{
  qDebug() << "onSave()";
  PatientData data;
  data.name = ui->nameLineEdit->text();
  data.surname = ui->surnameLineEdit->text();
  data.pesel = ui->peselLineEdit->text();
  emit savePatient(data);
}

void PatientForm::onClear()
{
  qDebug() << "PatientForm::onClear()";
  clear();
}

void PatientForm::clear()
{
  ui->nameLineEdit->clear();
  ui->surnameLineEdit->clear();
  ui->peselLineEdit->clear();
}

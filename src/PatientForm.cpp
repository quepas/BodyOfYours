#include "PatientForm.h"
#include "PatientData.h"

#include <QDebug>

PatientForm::PatientForm(QWidget *parent) 
  : Form(parent)
{
  name = new QLineEdit(this);
  surname = new QLineEdit(this);
  pesel = new QLineEdit(this);
  form_->addRow(tr("Imie"), name);
  form_->addRow(tr("Nazwisko"), surname);
  form_->addRow(tr("PESEL"), pesel);
  connect(buttons_, SIGNAL(buttonClicked(int)), this, SLOT(onButtonClicked(int)));
}

PatientForm::~PatientForm()
{
  delete name;
  delete surname;
  delete pesel;
}

void PatientForm::fill(const PatientData& data)
{
  name->setText(data.name);
  surname->setText(data.surname);
  pesel->setText(data.pesel);
  form_widget_->setEnabled(false);
}

void PatientForm::clear()
{
  name->clear();
  surname->clear();
  pesel->clear();
}

void PatientForm::onButtonClicked(int button)
{
  switch (button) {
  case FormButtons::SAVE: {
    PatientData data;
    data.name = name->text();
    data.surname = surname->text();
    data.pesel = pesel->text();
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

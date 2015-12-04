#include "PatientForm.h"
#include "PatientData.h"
#include "GuiActions.h"

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
  id = data.id;
  name->setText(data.name);
  surname->setText(data.surname);
  pesel->setText(data.pesel);
  setEnabled(false);
  buttons_->setShowFormState();
}

void PatientForm::clear()
{
  name->clear();
  surname->clear();
  pesel->clear();
  setEnabled(true);
  buttons_->setNewFormState();
}

void PatientForm::onButtonClicked(int button)
{
  switch (button) {
  case FormButtons::SAVE: {
    emit savePatient(getData());
    setEnabled(false);
    break;
  }
  case FormButtons::MODIFY:
    setEnabled(false);
    emit modifyPatient(getData());
    break;
  case FormButtons::CLEAR:
    clear();
    break;
  case FormButtons::CANCEL:
    break;
  case FormButtons::REMOVE:
    clear();
    ActionHub::trigger(ActionDeleteCurrentItem::TYPE());
    break;
  case FormButtons::LOCK:
    setEnabled(false);
    break;
  case FormButtons::UNLOCK:
    setEnabled(true);
    break;
  default:
    break;
  }
}

PatientData PatientForm::getData()
{
  PatientData data;
  data.id = id;
  data.name = name->text();
  data.surname = surname->text();
  data.pesel = pesel->text();
  return data;
}

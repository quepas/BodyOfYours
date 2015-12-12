#include "PatientForm.h"

PatientForm::PatientForm(QSqlTableModel* model, QWidget* parent) : FormWidget(model, parent)
{
  name_ = new QLineEdit(this);
  surname_ = new QLineEdit(this);
  pesel_ = new QLineEdit(this);
  formLayout_->addRow(tr("Imie"), name_);
  formLayout_->addRow(tr("Nazwisko"), surname_);
  formLayout_->addRow(tr("PESEL"), pesel_);

  mapper_->addMapping(name_, model_->fieldIndex("name"));
  mapper_->addMapping(surname_, model_->fieldIndex("surname"));
  mapper_->addMapping(pesel_, model_->fieldIndex("pesel"));
  setCurrentRowIndex(0);
}

PatientForm::~PatientForm()
{
  delete name_;
  delete surname_;
  delete pesel_;
};

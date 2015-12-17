#include "ExaminationForm.h"

ExaminationForm::ExaminationForm(QSqlTableModel* model, QWidget* parent) : FormWidget(model, parent)
{
  examName_ = new QLineEdit(this);
  formLayout_->addRow(tr("Nazwa badania"), examName_);

  mapper_->addMapping(examName_, model_->fieldIndex("name"));
  setCurrentRowIndex(0);
}

ExaminationForm::~ExaminationForm()
{
  delete examName_;
}
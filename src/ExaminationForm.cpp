#include "ExaminationForm.h"

ExaminationForm::ExaminationForm(QSqlTableModel* model, QWidget* parent) : FormWidget(model, parent)
{
  examName_ = new QLineEdit(this);
  scanName_ = new QLineEdit(this);
  formLayout_->addRow(tr("Nazwa badania"), examName_);
  formLayout_->addRow(tr("Plik skanu"), scanName_);

  mapper_->addMapping(examName_, model_->fieldIndex("name"));
  mapper_->addMapping(scanName_, model_->fieldIndex("scan_name"));
  setCurrentRowIndex(0);
}

ExaminationForm::~ExaminationForm()
{
  delete examName_;
  delete scanName_;
}
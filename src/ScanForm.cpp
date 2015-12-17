#include "ScanForm.h"

ScanForm::ScanForm(QSqlTableModel* model, QWidget* parent)
  : FormWidget(model, parent)
{
  scanName_ = new QLineEdit(this);
  scanFilePath_ = new QLineEdit(this);
  formLayout_->addRow(tr("Nazwa skanu"), scanName_);
  formLayout_->addRow(tr("Plik skanu"), scanFilePath_);
  scanFilePath_->setEnabled(false);

  mapper_->addMapping(scanName_, model_->fieldIndex("name"));
  mapper_->addMapping(scanFilePath_, model_->fieldIndex("filename"));
  setCurrentRowIndex(0);
}

ScanForm::~ScanForm()
{
  delete scanName_;
  delete scanFilePath_;
}

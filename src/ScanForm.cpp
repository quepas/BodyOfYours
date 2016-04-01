#include "ScanForm.h"

#include <QDebug>

ScanForm::ScanForm(QSqlTableModel* model, QWidget* parent)
  : FormWidget(model, parent), scanDiffModel_(nullptr)
{
  scanName_ = new QLineEdit(this);
  scanFilePath_ = new QLineEdit(this);
  formLayout_->addRow(tr(":scan_name"), scanName_);
  formLayout_->addRow(tr(":scan_file"), scanFilePath_);
  scanFilePath_->setEnabled(false);

  mapper_->addMapping(scanName_, model_->fieldIndex("name"));
  mapper_->addMapping(scanFilePath_, model_->fieldIndex("filename"));
  setCurrentRowIndex(0);

  scanDiffTable_ = new QTableView();
  scanDiffTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  scanDiffTable_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  scanDiffTable_->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  connect(scanDiffTable_, &QTableView::doubleClicked, [=]{
    int scanDiffRow = scanDiffTable_->currentIndex().row();
    auto record = scanDiffModel_->record(scanDiffRow);
    emit displayMeshWithQualityMap(record.value("ref_id").toInt(), record.value("id").toInt());
  });
  formLayout_->addWidget(scanDiffTable_);
}

ScanForm::~ScanForm()
{
  delete scanName_;
  delete scanFilePath_;
  delete scanDiffTable_;
  delete scanDiffModel_;
}

void ScanForm::setCurrentRowWithId(int itemID)
{
  FormWidget::setCurrentRowWithId(itemID);
  setupScanDiffTable(itemID);
}

void ScanForm::setupScanDiffTable(int scanDiffID)
{
  delete scanDiffModel_;
  scanDiffModel_ = new QSqlTableModel;
  scanDiffModel_->setTable("scan_diff");
  scanDiffModel_->setFilter(QString("ref_id == %1").arg(scanDiffID));
  scanDiffModel_->select();
  if (scanDiffModel_->rowCount() > 0) {
    scanDiffModel_->setHeaderData(2, Qt::Horizontal, QObject::tr("Roznica z"));
    scanDiffModel_->setHeaderData(3, Qt::Horizontal, QObject::tr("Nazwa pliku"));
    scanDiffTable_->setVisible(true);
    scanDiffTable_->setModel(scanDiffModel_);
    scanDiffTable_->setColumnHidden(0, true);
    scanDiffTable_->setColumnHidden(1, true);
  }
  else {
    scanDiffTable_->setVisible(false);
  }
}

#include "StackedFormWidget.h"

#include <QDebug>
#include <QSqlError>
#include "PatientForm.h"
#include "ExaminationForm.h"
#include "ScanForm.h"
#include "ModelHelper.h"

StackedFormWidget::StackedFormWidget(QSqlTableModel* patient_model, QSqlTableModel* exam_model, QSqlTableModel* scan_model, QWidget* parent /*= nullptr*/) : QStackedWidget(parent), currentRowID_(-1)
{
  widgets_.append(new PatientForm(patient_model, this));
  widgets_.append(new ExaminationForm(exam_model, this));
  widgets_.append(new FormWidget(this));
  widgets_.append(new ScanForm(scan_model, this));
  for (int i = 0; i < 2; ++i) {
    connect(widgets_[i], &FormWidget::canceled, [=]{
      switchTo(EMPTY_FORM);
    });
  }
  connect(dynamic_cast<ScanForm*>(widgets_[3]), &ScanForm::displayMeshWithQualityMap, [=] (int refScanID, int diffID) {
    emit displayMeshWithQualityMap(refScanID, diffID);
  });

  connect(widgets_[0], &FormWidget::deleted, [=](int deletedItemId) {
    ModelHelper::deletePatientRemains(deletedItemId, exam_model);
    switchTo(EMPTY_FORM);
  });
  connect(widgets_[1], &FormWidget::deleted, [=]{
    switchTo(EMPTY_FORM);
  });
  connect(widgets_[0], &FormWidget::saved, [=](int currentRowIndex) {
    switchTo(EMPTY_FORM);
  });
  connect(widgets_[1], &FormWidget::saved, [=](int currentRowIndex) {
    if (currentIndex() == EXAMINATION_FORM && currentRowID_ != -1) {
      auto idx = exam_model->index(currentRowIndex, exam_model->fieldIndex("patient_id"));
      exam_model->setData(idx, currentRowID_);
    }
    switchTo(EMPTY_FORM);
  });

  addWidget(widgets_[0]);
  addWidget(widgets_[1]);
  addWidget(widgets_[2]);
  addWidget(widgets_[3]);
  switchTo(EMPTY_FORM);
}

StackedFormWidget::~StackedFormWidget()
{
  for (int i = 0; i < widgets_.size(); ++i) {
    delete widgets_[i];
  }
}

void StackedFormWidget::switchTo(FormID id, int dataRowId)
{
  currentRowID_ = dataRowId;
  setCurrentIndex(id);
  widgets_[id]->setCurrentRowWithId(currentRowID_);
}

void StackedFormWidget::switchTo(FormID id)
{
  setCurrentIndex(id);
  if (id != EMPTY_FORM) widgets_[id]->addRow();
}

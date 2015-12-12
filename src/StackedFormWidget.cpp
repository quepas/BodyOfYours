#include "StackedFormWidget.h"

#include "PatientForm.h"
#include "ExaminationForm.h"

StackedFormWidget::StackedFormWidget(QSqlTableModel* patient_model, QSqlTableModel* exam_model, QWidget* parent /*= nullptr*/) : QStackedWidget(parent)
{
  widgets_.append(new PatientForm(patient_model, this));
  widgets_.append(new ExaminationForm(exam_model, this));
  addWidget(widgets_[0]);
  addWidget(widgets_[1]);
}

StackedFormWidget::~StackedFormWidget()
{
  for (int i = 0; i < widgets_.size(); ++i) {
    delete widgets_[i];
  }
}

void StackedFormWidget::switchTo(FormID id, int dataRowId)
{
  setCurrentIndex(id);
  widgets_[id]->setCurrentRowWithId(dataRowId);
}

void StackedFormWidget::switchTo(FormID id)
{
  setCurrentIndex(id);
  widgets_[id]->addRow();
}

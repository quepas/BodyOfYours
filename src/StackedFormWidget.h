#pragma once

#include "FormWidget.h"
#include <QStackedWidget>

class StackedFormWidget : public QStackedWidget
{
  Q_OBJECT
public:
  StackedFormWidget(QSqlTableModel* patient_model, QSqlTableModel* exam_model, QWidget* parent = nullptr);
  ~StackedFormWidget();

  enum FormID{
    PATIENT_FORM = 0,
    EXAMINATION_FORM = 1
  };

  void switchTo(FormID id);
  void switchTo(FormID id, int dataRowId);
private:
  QList<FormWidget*> widgets_;
};

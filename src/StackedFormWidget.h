#pragma once

#include "FormWidget.h"
#include <QStackedWidget>

class StackedFormWidget : public QStackedWidget
{
  Q_OBJECT
public:
  StackedFormWidget(QSqlTableModel* patient_model, QSqlTableModel* exam_model, QSqlTableModel* scan_model, QWidget* parent = nullptr);
  ~StackedFormWidget();

  enum FormID {
    PATIENT_FORM = 0,
    EXAMINATION_FORM = 1,
    EMPTY_FORM = 2,
    SCAN_FORM = 3
  };

  void switchTo(FormID id);
  void switchTo(FormID id, int dataRowId);

  FormWidget* form(FormID id) { return widgets_[id]; }

signals:
  void showRefMeshWithQualityMap(QString refMeshFilename, QString qualityMapFilename);

private:
  QList<FormWidget*> widgets_;
  int currentRowID_;
};

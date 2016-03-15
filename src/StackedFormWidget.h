#pragma once

#include "FormWidget.h"
#include "SQLTableModelHandler.h"
#include <QStackedWidget>

class StackedFormWidget : public QStackedWidget
{
  Q_OBJECT
public:
  StackedFormWidget(SQLTableModelHandler handler, QWidget* parent = nullptr);
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
  void displayMeshWithQualityMap(int scanID, int diffID);
  void displayScan(int scanId, int diffId);

private:
  QList<FormWidget*> widgets_;
  int currentRowID_;
};

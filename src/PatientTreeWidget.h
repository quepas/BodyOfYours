#pragma once

#include "FormWidget.h"
#include "PatientData.h"
#include "ExaminationData.h"
#include "PatientTreeItem.h"
#include "StackedFormWidget.h"
#include "ScanViewer.h"
#include "SQLTableModelHandler.h"

#include <QTreeWidget>
#include <QList>
#include <QSqlTableModel>

class PatientTreeWidget : public QTreeWidget
{
  Q_OBJECT
public:
  PatientTreeWidget(SQLTableModelHandler handler, StackedFormWidget* form_widget, const QList<PatientData>& patients, QWidget* parent = 0);
  ~PatientTreeWidget();

private:
  void buildTreeFromModel(QSqlTableModel* patient_model, QSqlTableModel* exam_model);

  void saveExpanded();
  void restorExpanded();

  QList<int> patients_exp_;
  QList<int> exam_exp_;

  StackedFormWidget* form_widget_;
  SQLTableModelHandler handler_;

public slots:
  void removeCurrentItem();
  void showCurrentScan();
  /*void showScan(int scanID);*/
  void onItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onDataChanged();

signals:
  void openScan(QString filename);
  void displayScan(ScanViewer::ID viewerID, int scanID);
  void showTabWithIndex(int);
};

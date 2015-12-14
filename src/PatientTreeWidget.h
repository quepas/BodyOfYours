#pragma once

#include "FormWidget.h"
#include "PatientData.h"
#include "ExaminationData.h"
#include "PatientTreeItem.h"
#include "StackedFormWidget.h"

#include <QTreeWidget>
#include <QList>
#include <QSqlTableModel>

class PatientTreeWidget : public QTreeWidget
{
  Q_OBJECT

public:
  PatientTreeWidget(QSqlTableModel* patient_model, QSqlTableModel* exam_model, StackedFormWidget* form_widget, const QList<PatientData>& patients, QWidget* parent = 0);
  ~PatientTreeWidget();

private:
  void buildTree(const QList<PatientData>& patients);
  void buildTreeFromModel(QSqlTableModel* patient_model, QSqlTableModel* exam_model);

  QTreeWidgetItem* addPatient(PatientData data);
  QTreeWidgetItem* modifyPatient(PatientData data);
  QTreeWidgetItem* addExamination(QTreeWidgetItem* parent, ExaminationData data);

  StackedFormWidget* form_widget_;
  QSqlTableModel* patient_model_;
  QSqlTableModel* exam_model_;
  QSqlTableModel* scan_model_;

public slots:
  void removeCurrentItem();

  void showScan();

  void onSavePatient(PatientData data);
  void onModifyPatient(PatientData data);
  void onSaveExamination(ExaminationData data);
  void onDeletePatient();

  void onItemDoubleClicked(QTreeWidgetItem* item, int column);

  void onDataChanged();

signals:
  void openScan(QString filename);
  void showTabWithIndex(int);
};

#pragma once

#include "FormWidget.h"
#include "PatientData.h"
#include "ExaminationData.h"
#include "PatientTreeItem.h"
#include "StackedFormWidget.h"

#include <QTreeWidget>
#include <QList>

class PatientTreeWidget : public QTreeWidget
{
  Q_OBJECT

public:
  PatientTreeWidget(StackedFormWidget* form_widget, const QList<PatientData>& patients, QWidget* parent = 0);
  ~PatientTreeWidget();

private:
  void buildTree(const QList<PatientData>& patients);

  QTreeWidgetItem* addPatient(PatientData data);
  QTreeWidgetItem* modifyPatient(PatientData data);
  QTreeWidgetItem* addExamination(QTreeWidgetItem* parent, ExaminationData data);

  StackedFormWidget* form_widget_;

public slots:
  void removeCurrentItem();

  void showScan();

  void onSavePatient(PatientData data);
  void onModifyPatient(PatientData data);
  void onSaveExamination(ExaminationData data);
  void onDeletePatient();

  void onItemDoubleClicked(QTreeWidgetItem* item, int column);

signals:
  void openScan(QString filename);
  void showTabWithIndex(int);
};

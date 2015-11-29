#pragma once

#include "FormViewer.h"
#include "PatientData.h"
#include "ExaminationData.h"
#include "PatientWidgetItem.h"

#include <QTreeWidget>
#include <QList>

class PatientWidget : public QTreeWidget
{
  Q_OBJECT

public:
  PatientWidget(FormViewer* form_viewer, const QList<PatientData>& patients, QWidget* parent = 0);
  ~PatientWidget();

private:
  void buildTree(const QList<PatientData>& patients);

  QTreeWidgetItem* addPatient(PatientData data);
  QTreeWidgetItem* addExamination(QTreeWidgetItem* parent, ExaminationData data);

  FormViewer* form_viewer_;

public slots:
  void removeCurrentItem();

  void showScan();

  void onSavePatient(PatientData data);
  void onSaveExamination(ExaminationData data);
  void onDeletePatient();

  void onItemDoubleClicked(QTreeWidgetItem* item, int column);

signals:
  void openScan(QString filename);
  void showTabWithIndex(int);
};

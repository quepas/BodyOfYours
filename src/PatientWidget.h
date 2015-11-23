#pragma once

#include "PatientData.h"
#include "ExaminationData.h"
#include "PatientWidgetItem.h"

#include <QTreeWidget>
#include <QList>

class PatientWidget : public QTreeWidget
{
 Q_OBJECT

public:
  PatientWidget(const QList<PatientData>& patients, QWidget* parent = 0);
  ~PatientWidget();

private:
  void buildTree(const QList<PatientData>& patients);

  QTreeWidgetItem* addPatient(PatientData data);
  QTreeWidgetItem* addExamination(QTreeWidgetItem* parent, ExaminationData data);

  void removeCurrentItem();

public slots:
  void showAddExaminationDialog();
  void removePatient();
  void removeExamination();

  void showIndex();
  void showScan();

  void onSavePatient(PatientData data);
  void onSaveExamination(ExaminationData data);
  void onDeletePatient();

  void onItemDoubleClicked(QTreeWidgetItem* item, int column);

signals:
  void openScan(QString filename);
};

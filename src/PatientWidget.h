#pragma once

#include "PatientItem.h"
#include <QTreeWidget>
#include <QList>

class PatientWidget : public QTreeWidget
{
 Q_OBJECT

public:
  PatientWidget(const QList<PatientData>& patients, QWidget* parent = 0);
  ~PatientWidget();

  void buildTree(const QList<PatientItem*>& patients);

  void addPatient();
  void addExamination();

public slots:
  void showAddPatientDialog();
  void showAddExaminationDialog();
  void removePatient();
  void removeExamination();

  void showIndex();
  void showScan();

  void onSavePatient(QString name);
  void onSaveExamination(ExaminationData data);

  void onItemDoubleClicked(QTreeWidgetItem* item, int column);

signals:
  void openScan(QString filename);
};

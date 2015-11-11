#pragma once

#include "PatientItem.h"
#include <QTreeWidget>

static const int SCAN_ITEM = QTreeWidgetItem::UserType + 2;

class PatientsWidget : public QTreeWidget
{
 Q_OBJECT

public:
  PatientsWidget(QWidget* parent = 0);
  ~PatientsWidget();

  void buildTree(const QList<PatientItem*>& patients);

  void addPatient();
  void addExamination();

  static QList<PatientItem*> prepareTestData();
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

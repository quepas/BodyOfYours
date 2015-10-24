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

  static QList<PatientItem*> prepareTestData();
public slots:
  void showAddPatientDialog();
  void showAddExaminationDialog();
  void removePatient();

  void showIndex();

  void onSavePatient(QString name);
};

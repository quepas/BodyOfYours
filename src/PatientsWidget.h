#pragma once

#include <QTreeWidget>

static const int PATIENT_ITEM = QTreeWidgetItem::UserType;
static const int EXAMINATION_ITEM = QTreeWidgetItem::UserType + 1;
static const int SCAN_ITEM = QTreeWidgetItem::UserType + 2;

class PatientsWidget : public QTreeWidget
{
 Q_OBJECT

public:
  PatientsWidget(QWidget* parent = 0);
  ~PatientsWidget();

public slots:
  void showAddPatientDialog();
  void removePatient();

  void onSavePatient(QString name);
};

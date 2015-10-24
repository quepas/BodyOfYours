#pragma once

#include <QTreeWidget>

static const int SCAN_ITEM = QTreeWidgetItem::UserType + 2;

class PatientsWidget : public QTreeWidget
{
 Q_OBJECT

public:
  PatientsWidget(QWidget* parent = 0);
  ~PatientsWidget();

public slots:
  void showAddPatientDialog();
  void showAddExaminationDialog();
  void removePatient();

  void showIndex();

  void onSavePatient(QString name);
};

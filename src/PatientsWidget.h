#pragma once

#include <QTreeWidget>

class PatientsWidget : public QTreeWidget
{
 Q_OBJECT

public:
  PatientsWidget(QWidget* parent = 0);
  ~PatientsWidget();

public slots:
  void showAddPatientDialog();
  void onSavePatient(QString name);
};

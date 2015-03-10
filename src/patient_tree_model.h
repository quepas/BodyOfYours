#pragma once

#include "data/patient.h"

#include <QStandardItemModel>
#include <QFileSystemModel>
#include <QVector>

class PatientTreeModel : public QStandardItemModel
{
public:
  PatientTreeModel(QObject* parent);

  bool Create(Patient data);
  void ReadAll();
  void Read(const QString& patient_id);
  bool Update(Patient data);
  void Delete(const QString& patient_id);

  void Build();

  QVector<Patient> patients() const { return patients_; }

private:
  QVector<Patient> patients_;
};

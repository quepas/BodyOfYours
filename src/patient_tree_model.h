#pragma once

#include "data/patient.h"

#include <QStandardItemModel>
#include <QString>
#include <QVector>

class PatientTreeModel : public QStandardItemModel
{
public:
  PatientTreeModel(QString root_path, QObject* parent = nullptr);

  bool Create(Patient data);
  void ReadAll();
  void Read(const QString& patient_id);
  bool Update(Patient data);
  void Delete(const QString& patient_id);

  void Build();

  QVector<Patient> patients() const { return patients_; }

private slots:
  void OnRebuild();

private:
  QString root_path_;
  QVector<Patient> patients_;
};

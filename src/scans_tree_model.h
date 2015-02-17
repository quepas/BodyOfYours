#pragma once

#include "patient_data.h"

#include <QStandardItemModel>
#include <QFileSystemModel>

class ScansTreeModel : public QStandardItemModel
{
public:
  ScansTreeModel(QObject* parent);

  void RemovePatient(const QModelIndex& index);

  void AddPatientToTree(PatientData data);
  void RemovePatientFromTree(const QModelIndex& index);

  void SavePatientToDisc(PatientData data);
  void LoadPatientFromDisc(QString name);
  void RemovePatientFromDisc(QString name);

  void SavePatientMetadata(PatientData data);
  PatientData LoadPatientMetadata(QString metadata_file);

private:
  QFileSystemModel* help_model_;

  void PrepareTree();
};

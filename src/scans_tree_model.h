#pragma once

#include "patient_data.h"

#include <QStandardItemModel>
#include <QFileSystemModel>

class ScansTreeModel : public QStandardItemModel
{
public:
  ScansTreeModel(QObject* parent);

  void AddPatientToTree(PatientData data);
  void AddPatientToDisc(PatientData data);
  void SavePatientMetadata(PatientData data);
private:
  QFileSystemModel* help_model_;

  void PrepareTree();
};

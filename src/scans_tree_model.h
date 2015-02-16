#pragma once

#include "patient_data.h"

#include <QStandardItemModel>
#include <QFileSystemModel>

class ScansTreeModel : public QStandardItemModel
{
public:
  ScansTreeModel(QObject* parent);

  void AddPatient(PatientData data);
private:
  QFileSystemModel* help_model_;

  void PrepareTree();
};

#pragma once

#include <QSqlRecord>
#include <QSqlTableModel>

class ModelHelper
{
public:
  static void deletePatientRemains(int id, QSqlTableModel* exam_model) {
    for (int i = 0; i < exam_model->rowCount(); ++i) {
      QSqlRecord record = exam_model->record(i);
      if (record.value("patient_id") == id) {
        exam_model->removeRow(i);
        // TODO: delete exam's scans
      }
    }
  }
  //static void deleteExaminationFromPatient(int patient_id);
};

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

  static bool findScanFilename(int scanID, QString& outFilename) {
    QSqlTableModel scanModel;
    scanModel.setTable("scan");
    scanModel.setFilter(QString("id == %1").arg(scanID));
    scanModel.select();
    if (scanModel.rowCount() != 1) return false;
    outFilename = scanModel.record(0).value("filename").toString();
    return true;
  }

  static bool findScanDiffFilename(int diffId, QString& outFilename) {
    QSqlTableModel scanDiffModel;
    scanDiffModel.setTable("scan_diff");
    scanDiffModel.setFilter(QString("id == %1").arg(diffId));
    scanDiffModel.select();
    if (scanDiffModel.rowCount() != 1) return false;
    outFilename = scanDiffModel.record(0).value("filename").toString();
    return true;
  }
};

#pragma once

#include <QDebug>
#include <QFile>
#include <QSqlRecord>
#include <QSqlTableModel>

class ModelHelper
{
public:
  static void deletePatient(int patient_id, SQLTableModelHandler handler) {
    for (int i = 0; i < handler.patient->rowCount(); ++i) {
      QSqlRecord record = handler.patient->record(i);
      if (record.value("id").toInt() == patient_id) {
        handler.patient->removeRow(i);
        break;
      }
    }
    deleteExaminations(patient_id, handler);
  }

  static void deleteExaminations(int patient_id, SQLTableModelHandler handler) {
    for (int i = 0; i < handler.examination->rowCount(); ++i) {
      QSqlRecord record = handler.examination->record(i);
      if (record.value("patient_id").toInt() == patient_id) {
        int exam_id = record.value("id").toInt();
        deleteScans(exam_id, handler);
        handler.examination->removeRow(i);
      }
    }
  }

  static void deleteScans(int examination_id, SQLTableModelHandler handler) {
    for (int i = 0; i < handler.scan->rowCount(); ++i) {
      QSqlRecord record = handler.scan->record(i);
      if (record.value("exam_id").toInt() == examination_id) {
        int scan_id = record.value("id").toInt();
        QString filename = record.value("filename").toString();
        if (!QFile::remove(filename)) {
          qDebug() << "[ERROR@deleteScans] Couldn't delete scan file " << filename;
        }
        deleteScanDiffs(scan_id, handler);
        handler.scan->removeRow(i);
      }
    }
  }

  static void deleteScanDiffs(int scan_id, SQLTableModelHandler handler) {
    for (int i = 0; i < handler.scan_diff->rowCount(); ++i) {
      QSqlRecord record = handler.scan_diff->record(i);
      if (record.value("ref_id").toInt() == scan_id) {
        QString filename = record.value("filename").toString();
        if (!QFile::remove(filename)) {
          qDebug() << "[ERROR@deleteScanDiffs] Couldn't delete scan diff file " << filename;
        }
        handler.scan_diff->removeRow(i);
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

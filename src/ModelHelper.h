#pragma once

#include <QDebug>
#include <QFile>
#include <QSqlRecord>
#include <QSqlTableModel>

class ModelHelper
{
public:
  static void deletePatient(int patient_id, QSqlTableModel* patient_model, QSqlTableModel* exam_model, QSqlTableModel* scan_model, QSqlTableModel* scan_diff_model) {
    for (int i = 0; i < patient_model->rowCount(); ++i) {
      QSqlRecord record = patient_model->record(i);
      if (record.value("id").toInt() == patient_id) {
        patient_model->removeRow(i);
        break;
      }
    }
    deleteExaminations(patient_id, exam_model, scan_model, scan_diff_model);
  }

  static void deleteExaminations(int patient_id, QSqlTableModel* exam_model, QSqlTableModel* scan_model, QSqlTableModel* scan_diff_model) {
    for (int i = 0; i < exam_model->rowCount(); ++i) {
      QSqlRecord record = exam_model->record(i);
      if (record.value("patient_id").toInt() == patient_id) {
        int exam_id = record.value("id").toInt();
        deleteScans(exam_id, scan_model, scan_diff_model);
        exam_model->removeRow(i);
      }
    }
  }

  static void deleteScans(int examination_id, QSqlTableModel* scan_model, QSqlTableModel* scan_diff_model) {
    for (int i = 0; i < scan_model->rowCount(); ++i) {
      QSqlRecord record = scan_model->record(i);
      if (record.value("exam_id").toInt() == examination_id) {
        int scan_id = record.value("id").toInt();
        QString filename = record.value("filename").toString();
        if (!QFile::remove(filename)) {
          qDebug() << "[ERROR@deleteScans] Couldn't delete scan file " << filename;
        }
        deleteScanDiffs(scan_id, scan_diff_model);
        scan_model->removeRow(i);
      }
    }
  }

  static void deleteScanDiffs(int scan_id, QSqlTableModel* scan_diff_model) {
    for (int i = 0; i < scan_diff_model->rowCount(); ++i) {
      QSqlRecord record = scan_diff_model->record(i);
      if (record.value("ref_id").toInt() == scan_id) {
        QString filename = record.value("filename").toString();
        if (!QFile::remove(filename)) {
          qDebug() << "[ERROR@deleteScanDiffs] Couldn't delete scan diff file " << filename;
        }
        scan_diff_model->removeRow(i);
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

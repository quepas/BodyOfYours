#pragma once

#include <QtSql/QSqlDatabase>
#include <QVariant>
  
static int ID = QVariant::UserType + 1;
struct PatientData;
struct ExaminationData;

class Database
{
public:
  Database(QString db_name, QString db_type = "QSQLITE");

  void createScheme();
  bool hasScheme();

  static bool insertPatient(PatientData data);
  static bool insertPatient(PatientData in, PatientData& out);
  static bool deletePatient(int id);
  static QList<PatientData> selectPatient();
  static bool selectPatient(int id, PatientData& out);
  static bool selectLastPatient(PatientData& out);

  static bool insertExamination(ExaminationData in, ExaminationData& out);
  static bool deleteExamination(int id);
  static QList<ExaminationData> selectExamination(int patient_id);
  static bool selectExamination(int id, ExaminationData& out);
  static bool selectLastExamination(ExaminationData& out);
private:
  QSqlDatabase db_;
};
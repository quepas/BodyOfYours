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
  static bool deletePatient(int id);
  static QList<PatientData> selectPatient();

  static bool insertExamination(ExaminationData data);
  static bool deleteExamination(int id);
  static QList<ExaminationData> selectExamination(int patient_id);
private:
  QSqlDatabase db_;
};
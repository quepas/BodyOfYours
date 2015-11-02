#pragma once

#include <QtSql/QSqlDatabase>
#include <QVariant>
  
static int ID = QVariant::UserType + 1;
struct PatientData;

class Database
{
public:
  Database(QString db_name, QString db_type = "QSQLITE");

  void createScheme();
  bool hasScheme();

  static bool insertPatient(PatientData data);
  static bool deletePatient(int id);
  static QList<PatientData> selectPatient();
private:
  QSqlDatabase db_;
};
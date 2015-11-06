#include "Database.h"
#include "PatientItem.h"

#include <QSqlQuery>
#include <QDebug>

Database::Database(QString db_name, QString db_type /*= "QSQLITE"*/)
{
  db_ = QSqlDatabase::addDatabase(db_type);
  db_.setDatabaseName(db_name);
  if (!db_.open()) {
    qDebug() << "Couldn't open database " << db_name << " (" << db_type << ")";
  }
}

void Database::createScheme()
{
  QSqlQuery query;
  query.exec(
    "CREATE TABLE patient ("
    "id INTEGER PRIMARY KEY, "
    "name varchar(100))");
  query.exec("CREATE TABLE examination ("
    "id INTEGER PRIMARY KEY, "
    "name varchar(100),"
    "patient_id INTEGER,"
    "FOREIGN KEY(patient_id) REFERENCES patient(id))");
}

bool Database::insertPatient(PatientData data)
{
  QSqlQuery query;
  query.prepare("INSERT INTO patient (name) "
    "VALUES (:name)");
  query.bindValue(":name", data.name);
  return query.exec();
}

bool Database::deletePatient(int id)
{
  QSqlQuery query;
  query.prepare("DELETE FROM patient WHERE id = :id");
  query.bindValue(":id", id);
  return query.exec();
  // TODO: delete patient's examinations!
}

QList<PatientData> Database::selectPatient()
{
  QList<PatientData> list;
  QSqlQuery query("SELECT id, name FROM patient");
  while (query.next()) {
    PatientData patient;
    patient.id = query.value(0).toInt();
    patient.name = query.value(1).toString();
    list.push_back(patient);
  }
  return list;
}

bool Database::insertExamination(ExaminationData data)
{
  QSqlQuery query;
  query.prepare("INSERT INTO examination (name, patient_id) "
    "VALUES (:name, :patient_id)");
  query.bindValue(":name", data.name);
  query.bindValue(":patient_id", data.patient_id);
  return query.exec();
}

bool Database::deleteExamination(int id)
{
  QSqlQuery query;
  query.prepare("DELETE FROM examination WHERE id = :id");
  query.bindValue(":id", id);
  return query.exec();
}

QList<ExaminationData> Database::selectExamination(int patient_id)
{
  QList<ExaminationData> list;
  QSqlQuery query("SELECT id, patient_id, name FROM examination");
  while (query.next()) {
    ExaminationData exam;
    exam.id = query.value(0).toInt();
    exam.patient_id = query.value(1).toInt();
    exam.name = query.value(2).toString();
    list.push_back(exam);
  }
  return list;
}

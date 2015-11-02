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
  query.exec("CREATE TABLE patient ("
    "id INTEGER PRIMARY KEY, "
    "name varchar(100))");
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

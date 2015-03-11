#pragma once

#include "scan.h"
#include "../json.h"

#include <QString>
#include <QVector>

enum Sex
{
  FEMALE,
  MALE
};

class Patient
{
public:
  Patient();
  Patient(QString name, QString surname, QString additional_info);

  QString id() { return id_; }
  QString name() { return name_; }
  QString surname() { return surname_; }
  QString additional_info() { return additional_info_; }
  Sex sex() { return sex_; }
  QVector<Scan> scans() { return scans_; }

  void setId(QString id) { id_ = id; }
  void setName(QString name) { name_ = name; }
  void setSurname(QString surname) { surname_ = surname; }
  void setAdditionalInfo(QString additional_info) { additional_info_ = additional_info; }
  void setSex(Sex sex) { sex_ = sex; }
  void setScans(QVector<Scan> scans) { scans_ = scans; }

  QtJson::JsonObject AsJsonObject();

private:
  QString id_;
  QString name_;
  QString surname_;
  QString additional_info_;
  Sex sex_;
  QVector<Scan> scans_;
};
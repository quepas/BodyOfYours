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
  Patient(QtJson::JsonObject json);
  Patient(QString name, QString surname, QString additional_info);

  QString id() { return id_; }
  QString name() { return name_; }
  QString surname() { return surname_; }
  QString additional_info() { return additional_info_; }
  Sex sex() { return sex_; }
  QVector<Scan> scans() { return scans_; }

  void set_id(QString id) { id_ = id; }
  void set_name(QString name) { name_ = name; }
  void set_surname(QString surname) { surname_ = surname; }
  void set_additional_info(QString additional_info) { additional_info_ = additional_info; }
  void set_sex(Sex sex) { sex_ = sex; }
  void set_scans(QVector<Scan> scans) { scans_ = scans; }

  QtJson::JsonObject AsJsonObject();
  void FromJsonObject(QtJson::JsonObject json);

private:
  QString id_;
  QString name_;
  QString surname_;
  QString additional_info_;
  Sex sex_;
  QVector<Scan> scans_;
};
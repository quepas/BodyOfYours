#pragma once

#include <QString>

class Patient
{
public:
  Patient(QString name, QString surname, QString additional_info);

  QString id() { return id_; }
  QString name() { return name_; }
  QString surname() { return surname_; }
  QString additional_info() { additional_info_; }

  void setId(QString id) { id_ = id; }
  void setName(QString name) { name_ = name; }
  void setSurname(QString surname) { surname_ = surname; }
  QString setAdditionalInfo(QString additional_info) { additional_info_ = additional_info; }

private:
  QString id_;
  QString name_;
  QString surname_;
  QString additional_info_;
};
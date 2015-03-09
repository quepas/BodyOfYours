#pragma once

#include <QString>

enum Sex_
{
  FEMALE_,
  MALE_
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
  Sex_ sex() { return sex_; }

  void setId(QString id) { id_ = id; }
  void setName(QString name) { name_ = name; }
  void setSurname(QString surname) { surname_ = surname; }
  void setAdditionalInfo(QString additional_info) { additional_info_ = additional_info; }
  void setSex(Sex_ sex) { sex_ = sex; }

private:
  QString id_;
  QString name_;
  QString surname_;
  QString additional_info_;
  Sex_ sex_;
};
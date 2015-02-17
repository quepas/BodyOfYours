#pragma once

#include <QString>

enum Sex {
  FEMALE,
  MALE
};

struct PatientData
{
  QString name, additional;
  Sex sex;
};

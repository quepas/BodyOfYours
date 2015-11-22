#pragma once

#include <QString>

struct PatientData
{
  int id;
  QString name;
  QString surname;
  QString pesel;

  QString prepareLabel() {
    return name + " " + surname + " (" + pesel + ")";
  }
};

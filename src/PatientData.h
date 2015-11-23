#pragma once

#include "ExaminationData.h"
#include <QList>
#include <QString>

struct PatientData
{
  int id;
  QString name;
  QString surname;
  QString pesel;
  QList<ExaminationData> examinations;

  QString prepareLabel() {
    return name + " " + surname + " (" + pesel + ")";
  }
};

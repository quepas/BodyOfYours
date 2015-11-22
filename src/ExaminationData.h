#pragma once

#include <QString>

struct ExaminationData
{
  int id;
  int patient_id;
  QString name;
  QString scan_name;

  QString prepareLabel() {
    return name;
  }
};

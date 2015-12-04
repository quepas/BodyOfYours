#pragma once

#include <QString>

struct ExaminationData
{
  int id;
  int patient_id;
  QString name;
  QString scan_name;
  QString type; // BADANIE PODSTAWOWE, POSTAWA_SWOBODNA

  QString prepareLabel() {
    return name + " (" + scan_name + ")";
  }
};

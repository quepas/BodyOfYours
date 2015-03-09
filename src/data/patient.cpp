#include "patient.h"

Patient::Patient()
  : id_("none"), name_("none"), surname_("none"), additional_info_("none")
{

}

Patient::Patient(QString name, QString surname, QString additional_info)
  : name_(name), surname_(surname), additional_info_(additional_info)
{

}

#include "patient.h"

Patient::Patient()
  : id_("none"), name_("none"), surname_("none"), additional_info_("none")
{

}

Patient::Patient(QString name, QString surname, QString additional_info)
  : name_(name), surname_(surname), additional_info_(additional_info)
{

}

QtJson::JsonObject Patient::AsJsonObject()
{
  QtJson::JsonObject json;
  json["id"] = id_;
  json["name"] = name_;
  json["surname"] = surname_;
  json["additional"] = additional_info_;
  json["sex"] = (sex_ == FEMALE) ? "Female" : "Male";
  return json;
}

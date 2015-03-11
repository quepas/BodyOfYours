#include "patient.h"

Patient::Patient()
  : id_("none"), name_("none"), surname_("none"), additional_info_("none")
{

}

Patient::Patient(QtJson::JsonObject json)
{
  FromJsonObject(json);
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

void Patient::FromJsonObject(QtJson::JsonObject json)
{
  id_ = json["id"].toString();
  name_ = json["name"].toString();
  surname_ = json["surname"].toString();
  additional_info_ = json["additional"].toString();
  sex_ = json["sex"].toString() == "Female" ? FEMALE : MALE;
}

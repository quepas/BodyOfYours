#include "patient.h"

using QtJson::JsonObject;
using QtJson::JsonArray;

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

JsonObject Patient::AsJsonObject()
{
  JsonObject json;
  json["id"] = id_;
  json["name"] = name_;
  json["surname"] = surname_;
  json["additional"] = additional_info_;
  json["sex"] = (sex_ == FEMALE) ? "Female" : "Male";
  JsonArray array;
  foreach(Scan scan, scans_) {
    array.append(scan.AsJsonObject());
  }
  json["scans"] = array;
  return json;
}

void Patient::FromJsonObject(JsonObject json)
{
  id_ = json["id"].toString();
  name_ = json["name"].toString();
  surname_ = json["surname"].toString();
  additional_info_ = json["additional"].toString();
  sex_ = json["sex"].toString() == "Female" ? FEMALE : MALE;
  JsonArray array = json["scans"].toList();
  foreach(QVariant element, array) {
    scans_.push_back(Scan(element.toMap()));
  }
}

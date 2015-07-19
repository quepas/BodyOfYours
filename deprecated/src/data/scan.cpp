#include "scan.h"

using QtJson::JsonObject;

Scan::Scan()
  : name_("none"), filename_("none"), description_("none"), datetime_()
{

}

Scan::Scan(QtJson::JsonObject json)
{
  FromJsonObject(json);
}

QtJson::JsonObject Scan::AsJsonObject()
{
  JsonObject json;
  json["name"] = name_;
  json["filename"] = filename_;
  json["description"] = description_;
  json["datetime"] = datetime_.toString();
  return json;
}

void Scan::FromJsonObject(QtJson::JsonObject json)
{
  name_ = json["name"].toString();
  filename_ = json["filename"].toString();
  description_ = json["description"].toString();
  datetime_ = QDateTime::fromString(json["datetime"].toString());
}

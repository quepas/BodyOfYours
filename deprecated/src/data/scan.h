#pragma once

#include "../json.h"

#include <QDateTime>
#include <QString>

class Patient;

class Scan
{
public:
  Scan();
  Scan(QtJson::JsonObject json);

  void set_name(QString name) { name_ = name; }
  void set_filename(QString filename) { filename_ = filename; }
  void set_description(QString description) { description_ = description; }
  void set_datetime(QDateTime datetime) { datetime_ = datetime; }
  void set_owner(Patient* owner) { owner_ = owner; }

  QString name() { return name_; }
  QString filename() { return filename_; }
  QString description() { return description_; }
  QDateTime datetime() { return datetime_; }
  Patient* owner() { return owner_; }

  QtJson::JsonObject AsJsonObject();
  void FromJsonObject(QtJson::JsonObject json);
private:
  QString name_;
  QString filename_;
  QString description_;
  QDateTime datetime_;
  Patient* owner_;
};

#pragma once

#include <QDateTime>
#include <QString>

class Scan
{
public:
  Scan();

  void set_name(QString name) { name_ = name; }
  void set_filename(QString filename) { filename_ = filename; }
  void set_description(QString description) { description_ = description; }
  void set_datetime(QDateTime datetime) { datetime_ = datetime; }

  QString name() { return name_; }
  QString filename() { return filename_; }
  QString description() { return description_; }
  QDateTime datetime() { return datetime_; }

private:
  QString name_;
  QString filename_;
  QString description_;
  QDateTime datetime_;
};

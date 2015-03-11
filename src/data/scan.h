#pragma once

#include <QString>

class Scan
{
public:
  Scan();

  void set_name(QString name) { name_ = name; }
  void set_filename(QString filename) { filename_ = filename; }

  QString name() { return name_; }
  QString filename() { return filename_; }

private:
  QString name_;
  QString filename_;
};

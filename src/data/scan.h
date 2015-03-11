#pragma once

#include <QString>

class Scan
{
public:
  Scan();

  void setName(QString name) { name_ = name; }
  void setFilename(QString filename) { filename_ = filename; }

  QString name() { return name_; }
  QString filename() { return filename_; }

private:
  QString name_;
  QString filename_;
};

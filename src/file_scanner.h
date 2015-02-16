#pragma once

#include <QString>
#include <QStringList>

class FileScanner
{
public:
  explicit FileScanner(QString current_dir = ".");
  ~FileScanner();

  QStringList ScanTopDirsName();

private:
  QString current_dir_;

};

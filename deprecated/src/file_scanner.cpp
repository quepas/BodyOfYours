#include "file_scanner.h"

#include <QDirIterator>

FileScanner::FileScanner(QString current_dir /*= "."*/)
  : current_dir_(current_dir)
{

}

FileScanner::~FileScanner()
{

}

QStringList FileScanner::ScanTopDirsName()
{
  QStringList result;
  QDirIterator it(current_dir_, QDir::Dirs | QDir::NoDotAndDotDot);
  while (it.hasNext()) {
    it.next();
    result << it.fileName();
  }
  return result;
}

QStringList FileScanner::ScanFiles(QString extension /*= ""*/)
{
  QStringList result;
  QDirIterator it(current_dir_, QStringList() << extension, QDir::Files);
  while (it.hasNext()) {
    it.next();
    result << it.fileName();
  }
  return result;
}

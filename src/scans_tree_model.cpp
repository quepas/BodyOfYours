#include "scans_tree_model.h"
#include "file_scanner.h"
#include "resources.h"

#include <QString>
#include <QVariant>
#include <QStandardItem>
#include <QDebug>

ScansTreeModel::ScansTreeModel(QObject* parent)
  : QStandardItemModel(),
    help_model_(new QFileSystemModel)
{
  setHorizontalHeaderItem(0, new QStandardItem(QString("Catalogue")));
  setHorizontalHeaderItem(1, new QStandardItem(QString("Patient")));
  setHorizontalHeaderItem(2, new QStandardItem(QString("Scan")));
  PrepareTree();
}

void ScansTreeModel::PrepareTree()
{
  // scan for dirs and files
  FileScanner scanner("./data/");
  QStringList dirs = scanner.ScanTopDirsName();
  QStandardItem* root = invisibleRootItem();

  foreach(QString str, dirs) {
    QStandardItem* item = new QStandardItem(str);
    root->appendRow(item);
  }
  // init metafiles if none exsists

  // build tree

  // insert metadata
}

void ScansTreeModel::AddPatient(PatientData data)
{
  QStandardItem* root = invisibleRootItem();
  QStandardItem* item = new QStandardItem(data.name);
  QIcon icon = (data.sex == FEMALE) ? QIcon(Resources::ICON_FEMALE) : QIcon(Resources::ICON_MALE);
  item->setIcon(icon);
  root->appendRow(item);
}

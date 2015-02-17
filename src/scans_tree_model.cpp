#include "scans_tree_model.h"
#include "file_scanner.h"
#include "resources.h"
#include "json.h"

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

void ScansTreeModel::AddPatientToTree(PatientData data)
{
  QStandardItem* root = invisibleRootItem();
  QStandardItem* item = new QStandardItem(data.name);
  QIcon icon = (data.sex == FEMALE) ? QIcon(Resources::ICON_FEMALE) : QIcon(Resources::ICON_MALE);
  item->setIcon(icon);
  root->appendRow(item);
}

void ScansTreeModel::AddPatientToDisc(PatientData data)
{
  QDir dir(Resources::SCANS_DATA_PATH + "/" + data.name);
  if (!dir.exists()) {
    dir.mkpath(".");
  }
}

void ScansTreeModel::SavePatientMetadata(PatientData data)
{
  QtJson::JsonObject patient;
  patient["name"] = data.name;
  patient["sex"] = (data.sex == FEMALE) ? "Female" : "Male";
  patient["additional"] = data.additional;

  QFile metadata_file(Resources::SCANS_DATA_PATH + "/" + data.name + ".json");
  metadata_file.open(QIODevice::WriteOnly);
  metadata_file.write(QtJson::serialize(patient));
  metadata_file.close();
}

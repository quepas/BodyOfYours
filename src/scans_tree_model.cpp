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
  setHorizontalHeaderItem(0, new QStandardItem(QString("Patient")));
  PrepareTree();
}

void ScansTreeModel::PrepareTree()
{
  // scan for dirs and files
  FileScanner scanner("./data/");
  QStringList dirs = scanner.ScanTopDirsName();

  foreach(QString str, dirs) {
    LoadPatientFromDisc(str);
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
  item->setEditable(false);
  root->appendRow(item);
}

void ScansTreeModel::SavePatientToDisc(PatientData data)
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

void ScansTreeModel::LoadPatientFromDisc(QString name)
{
  QDir dir(Resources::SCANS_DATA_PATH + "/" + name);
  if (dir.exists()) {
    PatientData data = LoadPatientMetadata(Resources::SCANS_DATA_PATH + "/" + name + ".json");
    AddPatientToTree(data);
  }
}

PatientData ScansTreeModel::LoadPatientMetadata(QString metadata_path)
{
  QFile metadata_file(metadata_path);
  metadata_file.open(QFile::ReadOnly | QFile::Text);
  QTextStream in_stream(&metadata_file);
  QtJson::JsonObject json = QtJson::parse(in_stream.readAll()).toMap();
  PatientData result;
  result.name = json["name"].toString();
  result.additional = json["additional"].toString();
  result.sex = (json["sex"].toString() == "Female") ? FEMALE : MALE;
  return result;
}

void ScansTreeModel::RemovePatientFromTree(const QModelIndex& index)
{
  QStandardItem* current = itemFromIndex(index);
  removeRow(current->row());
}

void ScansTreeModel::RemovePatientFromDisc(QString name)
{
  QDir path(Resources::SCANS_DATA_PATH + "/");
  path.remove(name + ".json");
  path.rmdir(name);
}

void ScansTreeModel::RemovePatient(const QModelIndex& index)
{
  RemovePatientFromDisc(itemFromIndex(index)->text());
  RemovePatientFromTree(index);
}

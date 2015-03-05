#include "patient_tree_model.h"
#include "file_scanner.h"
#include "resources.h"
#include "json.h"

#include <QString>
#include <QVariant>
#include <QStandardItem>
#include <QDebug>

PatientTreeModel::PatientTreeModel(QObject* parent)
  : QStandardItemModel(),
    help_model_(new QFileSystemModel)
{
  setHorizontalHeaderItem(0, new QStandardItem(QString("Patient")));
  PrepareTree();
}

void PatientTreeModel::PrepareTree()
{
  // scan for dirs and files
  FileScanner scanner("./data/");
  QStringList dirs = scanner.ScanTopDirsName();

  foreach(QString str, dirs) {
    LoadPatientFromDisc(str);
  }
}

void PatientTreeModel::AddPatientToTree(PatientData data, QStringList patient_scans /*= QStringList()*/)
{
  QStandardItem* root = invisibleRootItem();
  QStandardItem* patient_item = new QStandardItem(data.name);
  QIcon sex_icon = (data.sex == FEMALE) ? QIcon(Resources::ICON_FEMALE) : QIcon(Resources::ICON_MALE);
  QIcon scan_icon = QIcon(Resources::ICON_SCAN);
  patient_item->setIcon(sex_icon);
  patient_item->setEditable(false);
  foreach(QString scan_name, patient_scans) {
    QStandardItem* scan_item = new QStandardItem(scan_name);
    scan_item->setIcon(scan_icon);
    scan_item->setEditable(false);
    patient_item->appendRow(scan_item);
  }
  root->appendRow(patient_item);
  patient_data_.push_back(data);
}

void PatientTreeModel::SavePatientToDisc(PatientData data)
{
  QDir dir(Resources::SCANS_DATA_PATH + "/" + data.name);
  if (!dir.exists()) {
    dir.mkpath(".");
  }
}

void PatientTreeModel::SavePatientMetadata(PatientData data)
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

void PatientTreeModel::LoadPatientFromDisc(QString name)
{
  QString patient_path = Resources::SCANS_DATA_PATH + "/" + name;
  QDir dir(patient_path);
  if (dir.exists()) {
    PatientData data = LoadPatientMetadata(patient_path + ".json");
    QStringList scans = LoadPatientScansFromDisc(patient_path);
    AddPatientToTree(data, scans);
  }
}

PatientData PatientTreeModel::LoadPatientMetadata(QString metadata_path)
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

void PatientTreeModel::RemovePatientFromTree(const QModelIndex& index)
{
  QStandardItem* current = itemFromIndex(index);
  removeRow(current->row());
}

void PatientTreeModel::RemovePatientFromDisc(QString name)
{
  QDir path(Resources::SCANS_DATA_PATH + "/");
  path.remove(name + ".json");
  path.rmdir(name);
}

void PatientTreeModel::RemovePatient(const QModelIndex& index)
{
  RemovePatientFromDisc(itemFromIndex(index)->text());
  RemovePatientFromTree(index);
}

QStringList PatientTreeModel::LoadPatientScansFromDisc(QString patient_dir_path)
{
  FileScanner scanner(patient_dir_path);
  return scanner.ScanFiles("*.ply");
}

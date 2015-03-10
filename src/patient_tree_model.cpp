#include "patient_tree_model.h"
#include "file_scanner.h"
#include "resources.h"
#include "json.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QString>
#include <QVariant>
#include <QStandardItem>
#include <QDebug>

PatientTreeModel::PatientTreeModel(QString root_path, QObject* parent /*= nullptr*/)
  : QStandardItemModel(),
    root_path_(root_path)
{
  setHorizontalHeaderItem(0, new QStandardItem(QString("Patient")));
  ReadAll();
  Build();
}

bool PatientTreeModel::Create(Patient data)
{
  //Calculate Patient_ID = md5(datetime + patient:name)
  QString raw_id = QDateTime::currentDateTime().toString() + data.name() + data.surname();
  QString patient_id = QString(QCryptographicHash::hash(raw_id.toAscii(), QCryptographicHash::Md5).toHex());
  data.setId(patient_id);
  //Create directory ./data/patients/Patient_ID
  QDir patient_dir(root_path_ + patient_id);
  if (!patient_dir.exists()) {
    if (!patient_dir.mkpath(".")) return false;
  }
  //Create patient's metadata file in ./data/patients/Patient_ID/metadata.json
  QtJson::JsonObject patient;
  patient["id"] = data.id();
  patient["name"] = data.name();
  patient["surname"] = data.surname();
  patient["additional"] = data.additional_info();
  patient["sex"] = (data.sex() == FEMALE) ? "Female" : "Male";

  QFile metadata_file(root_path_ + patient_id + "/metadata.json");
  if (!metadata_file.open(QIODevice::WriteOnly)) return false;
  metadata_file.write(QtJson::serialize(patient));
  metadata_file.close();

  patients_.push_back(data);
  // Rebuild tree model
  clear();
  Build();
  return true;
}

void PatientTreeModel::ReadAll()
{
  FileScanner scanner(root_path_);
  QStringList dirs = scanner.ScanTopDirsName();

  foreach(QString str, dirs) {
    Read(str);
  }
}

void PatientTreeModel::Read(const QString& patient_id)
{
  QFile metadata_file(root_path_ + patient_id + "/metadata.json");
  metadata_file.open(QFile::ReadOnly | QFile::Text);
  QTextStream in_stream(&metadata_file);
  QtJson::JsonObject json = QtJson::parse(in_stream.readAll()).toMap();
  metadata_file.close();
  Patient patient;
  patient.setId(json["id"].toString());
  patient.setName(json["name"].toString());
  patient.setSurname(json["surname"].toString());
  patient.setAdditionalInfo(json["additional"].toString());
  patient.setSex((json["sex"].toString() == "Female") ? FEMALE : MALE);
  patients_.push_back(patient);
}

void PatientTreeModel::Build()
{
  QStandardItem* root = invisibleRootItem();
  foreach(Patient patient, patients_) {
    QStandardItem* patient_item = new QStandardItem(patient.name() + " " + patient.surname());
    QIcon sex_icon = (patient.sex() == FEMALE) ? QIcon(Resources::ICON_FEMALE) : QIcon(Resources::ICON_MALE);
    QIcon scan_icon = QIcon(Resources::ICON_SCAN);
    patient_item->setIcon(sex_icon);
    patient_item->setEditable(false);
    root->appendRow(patient_item);
  }
}

void PatientTreeModel::Delete(const QString& patient_id)
{
  QDir path(root_path_);
  path.remove(patient_id + "/metadata.json");
  path.rmdir(patient_id);
  clear();
  for (int i = 0; i < patients_.size(); ++i) {
    if (patients_[i].id() == patient_id) {
      patients_.remove(i);
      break;
    }
  }
  Build();
}

bool PatientTreeModel::Update(Patient patient)
{
  // Prepare JSON
  QtJson::JsonObject json;
  json["id"] = patient.id();
  json["name"] = patient.name();
  json["surname"] = patient.surname();
  json["additional"] = patient.additional_info();
  json["sex"] = (patient.sex() == FEMALE) ? "Female" : "Male";

  // Open metadata.json for write
  QFile metadata_file(root_path_ + patient.id() + "/metadata.json");
  if (!metadata_file.open(QIODevice::WriteOnly)) return false;
  metadata_file.seek(0);
  metadata_file.write(QtJson::serialize(json));
  metadata_file.close();

  // Rebuild model
  clear();
  patients_.clear();
  ReadAll();
  Build();

  return true;
}

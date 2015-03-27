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
  data.set_id(patient_id);
  //Create directory ./data/patients/Patient_ID
  QDir patient_dir(root_path_ + patient_id);
  if (!patient_dir.exists()) {
    if (!patient_dir.mkpath(".")) return false;
  }
  // Create directory for patient's scans ./data/patients/Patient_ID/scans
  QDir scans_dir(root_path_ + patient_id + "/scans");
  if (!scans_dir.exists()) {
    if (!scans_dir.mkpath(".")) return false;
  }
  //Create patient's metadata file in ./data/patients/Patient_ID/metadata.json
  QFile metadata_file(root_path_ + patient_id + "/metadata.json");
  if (!metadata_file.open(QIODevice::WriteOnly)) return false;
  metadata_file.write(QtJson::serialize(data.AsJsonObject()));
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
  // Read metadata file ./data/patients/Patient_ID/metadata.json
  QFile metadata_file(root_path_ + patient_id + "/metadata.json");
  metadata_file.open(QFile::ReadOnly | QFile::Text);
  QTextStream in_stream(&metadata_file);
  QtJson::JsonObject json = QtJson::parse(in_stream.readAll()).toMap();
  metadata_file.close();
  // Check scans in ./data/patients/Patient_ID/scans directory
  FileScanner scanner(root_path_ + patient_id + "/scans");
  QStringList scans_list = scanner.ScanFiles("*.ply");
  // Construct Patient instance from JSON data and scans data
  Patient patient(json);
  QVector<Scan> filtered_scans;
  auto scans = patient.scans();
  // Read scan data only if scan file exsists
  foreach(Scan scan, scans) {
    if (scans_list.contains(scan.filename())) {
      filtered_scans.push_back(scan);
    }
  }
  patient.set_scans(filtered_scans);
  patients_.push_back(patient);
}

void PatientTreeModel::Build()
{
  QStandardItem* root = invisibleRootItem();
  foreach(Patient patient, patients_) {
    int scans_num = patient.scans().size();
    QStandardItem* patient_item = new QStandardItem(patient.name() + " " + patient.surname() + " (" + QString::number(scans_num) + ")");
    QIcon sex_icon = (patient.sex() == FEMALE) ? QIcon(Resources::ICON_FEMALE) : QIcon(Resources::ICON_MALE);
    QIcon scan_icon = QIcon(Resources::ICON_SCAN);
    patient_item->setIcon(sex_icon);
    patient_item->setEditable(false);
    foreach(Scan scan, patient.scans()) {
      QStandardItem* scan_item = new QStandardItem(scan.filename());
      scan_item->setIcon(scan_icon);
      scan_item->setEditable(false);
      patient_item->appendRow(scan_item);
    }
    root->appendRow(patient_item);
  }
  setHorizontalHeaderItem(0, new QStandardItem(QString("Patient")));
}

void PatientTreeModel::Delete(const QString& patient_id)
{
  QDir path(root_path_);
  path.remove(patient_id + "/metadata.json");
  path.rmdir(patient_id + "/scans");
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
  // Open metadata.json for write
  QFile metadata_file(root_path_ + patient.id() + "/metadata.json");
  if (!metadata_file.open(QIODevice::WriteOnly)) return false;
  metadata_file.seek(0);
  metadata_file.write(QtJson::serialize(patient.AsJsonObject()));
  metadata_file.close();

  // Rebuild model
  clear();
  patients_.clear();
  ReadAll();
  Build();

  return true;
}

void PatientTreeModel::OnRebuild()
{
  //clear()
}

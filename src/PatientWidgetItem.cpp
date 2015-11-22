#include "PatientWidgetItem.h"

PatientWidgetItem::PatientWidgetItem(PatientWidgetItemType type, int id, QString label, QWidget* parent /*= nullptr*/)
  : QTreeWidgetItem(QStringList(label), type)
{
  setData(0, PatientWidgetItemData::ID, id);
}

PatientWidgetItem* PatientWidgetItem::createPatientItem(int id, QString label, QWidget* parent /*= nullptr*/)
{
  return new PatientWidgetItem(PATIENT, id, label, parent);
}

PatientWidgetItem* PatientWidgetItem::createExamItem(int id, QString label, QWidget* parent /*= nullptr*/)
{
  return new PatientWidgetItem(EXAM, id, label, parent);
}

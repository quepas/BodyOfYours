#include "PatientTreeItem.h"

PatientTreeItem::PatientTreeItem(PatientWidgetItemType type, int id, QString label, QWidget* parent /*= nullptr*/)
  : QTreeWidgetItem(QStringList(label), type)
{
  setData(0, PatientTreeItemData::ID, id);
}

PatientTreeItem* PatientTreeItem::createPatientItem(int id, QString label, QWidget* parent /*= nullptr*/)
{
  return new PatientTreeItem(PATIENT, id, label, parent);
}

PatientTreeItem* PatientTreeItem::createExamItem(int id, QString label, QWidget* parent /*= nullptr*/)
{
  return new PatientTreeItem(EXAM, id, label, parent);
}

bool PatientTreeItem::isPatient(QTreeWidgetItem* item)
{
  return item->type() == PATIENT;
}

bool PatientTreeItem::isExamination(QTreeWidgetItem* item)
{
  return item->type() == EXAM;
}

int PatientTreeItem::getId(QTreeWidgetItem* item)
{
  return item->data(0, ID).toInt();
}

void PatientTreeItem::setId(QTreeWidgetItem* item, int id)
{
  item->setData(0, ID, id);
}

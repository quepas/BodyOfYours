#pragma once

#include <QTreeWidgetItem>

enum PatientWidgetItemType
{
  PATIENT = QTreeWidgetItem::UserType + 1,
  EXAM = QTreeWidgetItem::UserType + 2,
  SCAN = QTreeWidgetItem::UserType + 3
};

enum PatientTreeItemData
{
  ID = QVariant::UserType + 1
};

class PatientTreeItem : public QTreeWidgetItem
{
public:
  static PatientTreeItem* createPatientItem(int id, QString label, QWidget* parent = nullptr);
  static PatientTreeItem* createExamItem(int id, QString label, QWidget* parent = nullptr);
  static PatientTreeItem* createScanItem(int id, QString label, QWidget* parent = nullptr);

  static bool isPatient(QTreeWidgetItem* item);
  static bool isExamination(QTreeWidgetItem* item);
  static bool isScan(QTreeWidgetItem* item);
  static int getId(QTreeWidgetItem* item);
  static void setId(QTreeWidgetItem* item, int id);
private:
  PatientTreeItem(PatientWidgetItemType type, int id, QString label, QWidget* parent = nullptr);

  QString label;
};

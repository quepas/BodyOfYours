#pragma once

#include <QTreeWidgetItem>

enum PatientWidgetItemType
{
  PATIENT = QTreeWidgetItem::UserType + 1,
  EXAM = QTreeWidgetItem::UserType + 2
};

enum PatientWidgetItemData
{
  ID = QVariant::UserType + 1
};

class PatientWidgetItem : public QTreeWidgetItem
{
public:
  static PatientWidgetItem* createPatientItem(int id, QString label, QWidget* patient = nullptr);
  static PatientWidgetItem* createExamItem(int id, QString label, QWidget* patient = nullptr);

  static bool isPatient(QTreeWidgetItem* item);
  static bool isExamination(QTreeWidgetItem* item);
  static int getId(QTreeWidgetItem* item);
  static void setId(QTreeWidgetItem* item, int id);
private:
  PatientWidgetItem(PatientWidgetItemType type, int id, QString label, QWidget* parent = nullptr);

  QString label;
};

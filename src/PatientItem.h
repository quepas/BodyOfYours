#pragma once

#include <QTreeWidgetItem>

static const int PATIENT_ITEM = QTreeWidgetItem::UserType;

class PatientItem : public QTreeWidgetItem
{
public:
  PatientItem(QString name);
  ~PatientItem();
};
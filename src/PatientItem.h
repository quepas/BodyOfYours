#pragma once

#include <QTreeWidgetItem>

class PatientItem : public QTreeWidgetItem
{
public:
  PatientItem(QString name);
  ~PatientItem();
};
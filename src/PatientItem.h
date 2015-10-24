#pragma once

#include "ExaminationItem.h"
#include <QTreeWidgetItem>

static const int PATIENT_ITEM = QTreeWidgetItem::UserType;

class PatientItem : public QTreeWidgetItem
{
public:
  PatientItem(QString name);
  ~PatientItem();

  QString name() { return name_; }
  void insertExaminations(const QList<ExaminationItem*>& examinations);

private:
  QList<ExaminationItem*> examinations_;
  QString name_;
};
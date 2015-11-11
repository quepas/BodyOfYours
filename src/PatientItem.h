#pragma once

#include "ExaminationItem.h"
#include <QTreeWidgetItem>

static const int PATIENT_ITEM = QTreeWidgetItem::UserType;

struct PatientData
{
  int id;
  QString name;
};

class PatientItem : public QTreeWidgetItem
{
public:
  PatientItem(int id, QString name);
  ~PatientItem();

  QString name() { return name_; }
  const QList<ExaminationItem*>& examinations() { return examinations_; }
  void insertExaminations(const QList<ExaminationItem*>& examinations) { examinations_ = examinations; }

private:
  QList<ExaminationItem*> examinations_;
  QString name_;
};
#pragma once

#include <QTreeWidgetItem>

static const int EXAMINATION_ITEM = QTreeWidgetItem::UserType + 1;

struct ExaminationData
{
  int id;
  int patient_id;
  QString name;
};

class ExaminationItem : public QTreeWidgetItem
{
public:
  ExaminationItem(int id, QString name);
  ~ExaminationItem();

  QString name() { return name_; }
private:
  QString name_;
};

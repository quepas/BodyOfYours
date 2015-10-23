#pragma once

#include <QTreeWidgetItem>

static const int EXAMINATION_ITEM = QTreeWidgetItem::UserType + 1;

class ExaminationItem : public QTreeWidgetItem
{
public:
  ExaminationItem(QString name);
  ~ExaminationItem();
};

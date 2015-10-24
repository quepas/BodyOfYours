#include "ExaminationItem.h"

ExaminationItem::ExaminationItem(QString name)
  : QTreeWidgetItem(QStringList(name), EXAMINATION_ITEM),
    name_(name)
{

}

ExaminationItem::~ExaminationItem()
{

}

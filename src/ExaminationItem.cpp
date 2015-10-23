#include "ExaminationItem.h"

ExaminationItem::ExaminationItem(QString name)
  : QTreeWidgetItem(QStringList(name), EXAMINATION_ITEM)
{

}

ExaminationItem::~ExaminationItem()
{

}

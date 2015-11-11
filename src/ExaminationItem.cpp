#include "ExaminationItem.h"
#include "Database.h"

ExaminationItem::ExaminationItem(int id, QString name)
  : QTreeWidgetItem(QStringList(name), EXAMINATION_ITEM),
    name_(name)
{
  setData(0, ID, id);
}

ExaminationItem::~ExaminationItem()
{

}

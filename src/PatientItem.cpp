#include "PatientItem.h"

PatientItem::PatientItem(QString name)
  : QTreeWidgetItem(QStringList(name))
{

}

PatientItem::~PatientItem()
{

}

#include "PatientItem.h"

PatientItem::PatientItem(QString name)
  : QTreeWidgetItem(QStringList(name), PATIENT_ITEM)
{
}

PatientItem::~PatientItem()
{

}

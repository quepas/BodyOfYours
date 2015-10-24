#include "PatientItem.h"

PatientItem::PatientItem(QString name)
  : QTreeWidgetItem(QStringList(name), PATIENT_ITEM),
    name_(name)
{
}

PatientItem::~PatientItem()
{

}

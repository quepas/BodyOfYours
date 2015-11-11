#include "PatientItem.h"
#include "Database.h"

PatientItem::PatientItem(int id, QString name)
  : QTreeWidgetItem(QStringList(name), PATIENT_ITEM),
    name_(name)
{
  setData(0, ID, id);
}

PatientItem::~PatientItem()
{

}

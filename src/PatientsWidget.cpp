#include "PatientsWidget.h"
#include "PatientItem.h"

#include <QDebug>

PatientsWidget::PatientsWidget(QWidget* parent /*= 0*/)
{

}

PatientsWidget::~PatientsWidget()
{

}

void PatientsWidget::addPatient()
{
  qDebug() << "PatientsWidget::addPatient(QString)";
  // display patient dialog
  addTopLevelItem(new PatientItem("Pacjent A"));
}

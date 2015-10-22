#include "PatientsWidget.h"
#include "PatientItem.h"
#include "PatientDialog.h"

#include <QDebug>

PatientsWidget::PatientsWidget(QWidget* parent /*= 0*/)
{

}

PatientsWidget::~PatientsWidget()
{

}

void PatientsWidget::showAddPatientDialog()
{
  auto dialog = new PatientDialog(this);
  connect(dialog, SIGNAL(savePatient(QString)), this, SLOT(onSavePatient(QString)));
  dialog->show();
}

void PatientsWidget::onSavePatient(QString name)
{
  addTopLevelItem(new PatientItem(name));
}

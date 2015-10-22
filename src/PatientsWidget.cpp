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

void PatientsWidget::removePatient()
{
  if (currentItem() != nullptr) {
    qDebug() << "PatientsWidget => \n\tcurrent patient: " << currentItem()->text(0);
    qDebug() << "\tcurrent index row: " << currentIndex().row();
    qDebug() << "\tcurrent index column: " << currentIndex().column();
    auto item = takeTopLevelItem(currentIndex().row());
    delete item;
  }
}

void PatientsWidget::onSavePatient(QString name)
{
  addTopLevelItem(new PatientItem(name));
}

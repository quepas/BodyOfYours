#include "PatientsWidget.h"
#include "PatientItem.h"
#include "ExaminationItem.h"
#include "PatientDialog.h"

#include <QDebug>
#include <QMessageBox>

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

void PatientsWidget::showAddExaminationDialog()
{
  auto current_item = currentItem();
  if (!current_item) {
    qDebug() << "No current item selected.";
    return;
  }
  if (current_item->type() != PATIENT_ITEM) {
    QMessageBox::information(this, "Nowe badanie", "W celu dodania badania zaznacz pacjenta.");
    return;
  }
  current_item->addChild(new ExaminationItem("Badanie"));
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

void PatientsWidget::showIndex()
{
  if (!currentItem()) {
    qDebug() << "No current item selected.";
    return;
  }
  qDebug() << "Current item:";
  qDebug() << "\ttext: " << currentItem()->text(0);
  qDebug() << "\trow: " << currentIndex().row();
  qDebug() << "\tcolumn: " << currentIndex().column();
}

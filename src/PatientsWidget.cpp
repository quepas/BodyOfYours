#include "PatientsWidget.h"
#include "PatientItem.h"
#include "ExaminationItem.h"
#include "PatientDialog.h"

#include <QDebug>
#include <QMessageBox>
#include <QSqlQuery>

PatientsWidget::PatientsWidget(QWidget* parent /*= 0*/)
{
  QSqlQuery query("SELECT name FROM patient");
  while (query.next()) {
    QString name = query.value(0).toString();
    addTopLevelItem(new PatientItem(name));
  }
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
    QString name = currentItem()->text(0);
    qDebug() << "PatientsWidget => \n\tcurrent patient: " << name;
    qDebug() << "\tcurrent index row: " << currentIndex().row();
    qDebug() << "\tcurrent index column: " << currentIndex().column();
    QSqlQuery query;
    query.prepare("DELETE FROM patient WHERE name = :name");
    query.bindValue(":name", name);
    query.exec();
    auto item = takeTopLevelItem(currentIndex().row());
    delete item;
  }
}

void PatientsWidget::onSavePatient(QString name)
{
  addTopLevelItem(new PatientItem(name));
  QSqlQuery query;
  query.prepare("INSERT INTO patient (id, name) "
    "VALUES (:id, :name)");
  query.bindValue(":id", 1);
  query.bindValue(":name", name);
  query.exec();
}

void PatientsWidget::showIndex()
{
  if (!currentItem()) {
    qDebug() << "No current item selected.";
    return;
  }
  qDebug() << "Current item:";
  qDebug() << "\ttext: " << currentItem()->text(0);
  if (currentItem()->type() == PATIENT_ITEM) {
    qDebug() << "\tpatient: " << currentIndex().row();
  }
  else if (currentItem()->type() == EXAMINATION_ITEM) {
    qDebug() << "\tpatient: " << indexOfTopLevelItem(currentItem()->parent());
    qDebug() << "\texamination: " << currentIndex().row();
  }
}

void PatientsWidget::buildTree(const QList<PatientItem*>& patients)
{
  for (auto& patient : patients) {
    addTopLevelItem(patient);
    for (auto& examination : patient->examinations()) {
      patient->addChild(examination);
    }
  }
}

QList<PatientItem*> PatientsWidget::prepareTestData()
{
  QList<PatientItem*> patients;
  auto patient_1 = new PatientItem("Pacjent 1");
  QList<ExaminationItem*> examinations;
  examinations.push_back(new ExaminationItem("Badanie 1"));
  examinations.push_back(new ExaminationItem("Badanie 2"));
  patient_1->insertExaminations(examinations);
  patients.push_back(patient_1);
  patients.push_back(new PatientItem("Pacjent 2"));
  patients.push_back(new PatientItem("Pacjent 3"));
  return patients;
}

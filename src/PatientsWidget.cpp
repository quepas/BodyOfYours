#include "PatientsWidget.h"
#include "PatientItem.h"
#include "ExaminationItem.h"
#include "PatientDialog.h"
#include "Database.h"

#include <QDebug>
#include <QMessageBox>

PatientsWidget::PatientsWidget(QWidget* parent /*= 0*/)
{
  auto patients = Database::selectPatient();
  for (auto& patient : patients) {
    auto pt = new PatientItem(patient.name);
    pt->setData(0, ID, patient.id);
    addTopLevelItem(pt);
    // add examinations
    auto exams = Database::selectExamination(patient.id);
    for (auto& exam : exams) {
      auto ex = new ExaminationItem(exam.name);
      ex->setData(0, ID, exam.id);
      pt->addChild(ex);
    }
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
    Database::deletePatient(currentItem()->data(0, ID).toInt());
    auto item = takeTopLevelItem(currentIndex().row());
    delete item;
  }
}

void PatientsWidget::onSavePatient(QString name)
{
  PatientData patient;
  patient.name = name;
  Database::insertPatient(patient);
  addTopLevelItem(new PatientItem(patient.name));
}

void PatientsWidget::showIndex()
{
  if (!currentItem()) {
    qDebug() << "No current item selected.";
    return;
  }
  qDebug() << "Current item:";
  qDebug() << "\ttext: " << currentItem()->text(0);
  qDebug() << "\tID: " << currentItem()->data(0, ID);
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

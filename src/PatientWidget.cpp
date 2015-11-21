#include "PatientWidget.h"
#include "PatientDialog.h"
#include "Database.h"
#include "examinationdialog.h"
#include "PatientWidgetItem.h"

#include <QTreeWidgetItem>

#include <QDebug>
#include <QMessageBox>

PatientWidget::PatientWidget(const QList<PatientData>& patients, QWidget* parent /*= 0*/)
{
  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  setHeaderLabels(QStringList(("Pacjent")));
  for (auto& patient : patients) {
    auto pt = PatientWidgetItem::createPatientItem(patient.id, patient.name);
    pt->setIcon(0, QIcon("icon/broken8.png"));
    addTopLevelItem(pt);
    // add examinations
    auto exams = Database::selectExamination(patient.id);
    for (auto& exam : exams) {
      auto ex = PatientWidgetItem::createExamItem(exam.id, exam.name);
      ex->setIcon(0, QIcon("icon/stethoscope1.png"));
      pt->addChild(ex);
    }
  }

  // init signal/slots
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
}

PatientWidget::~PatientWidget()
{

}

void PatientWidget::showAddPatientDialog()
{
  auto dialog = new PatientDialog(this);
  connect(dialog, SIGNAL(savePatient(QString)), this, SLOT(onSavePatient(QString)));
  dialog->show();
}

void PatientWidget::showAddExaminationDialog()
{
  auto current_item = currentItem();
  if (!current_item) {
    qDebug() << "No current item selected.";
    return;
  }
  if (current_item->type() != PATIENT) {
    QMessageBox::information(this, "Nowe badanie", "W celu dodania badania zaznacz pacjenta.");
    return;
  }
  current_item->addChild(PatientWidgetItem::createExamItem(0, "Badanie"));
  auto dialog = new ExaminationDialog();
  connect(dialog, SIGNAL(saveExamination(ExaminationData)), this, SLOT(onSaveExamination(ExaminationData)));
  dialog->show();
}

void PatientWidget::removePatient()
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

void PatientWidget::onSavePatient(QString name)
{
  PatientData patient;
  patient.name = name;
  PatientData saved;
  Database::insertPatient(patient, saved);
  addTopLevelItem(PatientWidgetItem::createPatientItem(saved.id, saved.name));
}

void PatientWidget::showIndex()
{
  if (!currentItem()) {
    qDebug() << "No current item selected.";
    return;
  }
  qDebug() << "Current item:";
  qDebug() << "\ttext: " << currentItem()->text(0);
  qDebug() << "\tID: " << currentItem()->data(0, ID);
  if (currentItem()->type() == PATIENT) {
    qDebug() << "\tpatient: " << currentIndex().row();
  }
  else if (currentItem()->type() == EXAM) {
    qDebug() << "\tpatient: " << indexOfTopLevelItem(currentItem()->parent());
    qDebug() << "\texamination: " << currentIndex().row();
  }
}

void PatientWidget::buildTree(const QList<PatientData*>& patients)
{
  for (auto& patient : patients) {
   /* addTopLevelItem(patient);
    for (auto& examination : patient->examinations()) {
      patient->addChild(examination);
    }*/
  }
}

void PatientWidget::onSaveExamination(ExaminationData data)
{
  ExaminationData saved;
  Database::insertExamination(data, saved);
  currentItem()->addChild(PatientWidgetItem::createExamItem(saved.id, data.name));
}

void PatientWidget::removeExamination()
{
  if (!currentItem()) {
    qDebug() << "No current item selected.";
    return;
  }
  if (currentItem()->type() != EXAM) {
    QMessageBox::information(this, "Usun badanie", "W celu usuniecia badania zaznacz badanie.");
    return;
  }
  auto item = currentItem();
  int id = item->data(0, ID).toInt();
  Database::deleteExamination(id);
  item->parent()->removeChild(item);
}

void PatientWidget::showScan()
{
  if (!currentItem()) {
    qDebug() << "[WARNING] No item currently selected.";
    return;
  }
  if (currentItem()->type() != EXAM) {
    QMessageBox::information(this, "Pokaz skan", "W celu otwarcia skanu zaznacz badanie.");
    return;
  }
  auto item = currentItem();
  int id = item->data(0, ID).toInt();
  ExaminationData out;
  Database::selectExamination(id, out);
  emit openScan(out.name);
}

void PatientWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
  qDebug() << "Double clicked: " << item->text(0) << " (" << item->data(0, ID) << ")";
}

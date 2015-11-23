#include "PatientWidget.h"
#include "Database.h"
#include "PatientWidgetItem.h"

#include <QTreeWidgetItem>

#include <QDebug>
#include <QMessageBox>

PatientWidget::PatientWidget(const QList<PatientData>& patients, QWidget* parent /*= 0*/)
{
  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  setHeaderLabels(QStringList(("Pacjent")));

  buildTree(patients);

  // init signal/slots
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
}

PatientWidget::~PatientWidget()
{

}

void PatientWidget::showAddExaminationDialog()
{
  auto current_item = currentItem();
  if (!current_item) {
    qDebug() << "No current item selected.";
    return;
  }
  if (!PatientWidgetItem::isPatient(current_item)) {
    QMessageBox::information(this, "Nowe badanie", "W celu dodania badania zaznacz pacjenta.");
    return;
  }
  /*auto dialog = new ExaminationDialog();
  connect(dialog, SIGNAL(saveExamination(ExaminationData)), this, SLOT(onSaveExamination(ExaminationData)));
  dialog->show();*/
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

void PatientWidget::onSavePatient(PatientData data)
{
  PatientData saved;
  Database::insertPatient(data, saved);
  addPatient(saved);
}

void PatientWidget::showIndex()
{
  auto item = currentItem();
  if (!item) {
    qDebug() << "No current item selected.";
    return;
  }
  qDebug() << "Current item:";
  qDebug() << "\ttext: " << item->text(0);
  qDebug() << "\tID: " << PatientWidgetItem::getId(item);
  if (PatientWidgetItem::isPatient(item)) {
    qDebug() << "\tpatient: " << currentIndex().row();
  }
  else if (PatientWidgetItem::isExamination(item)) {
    qDebug() << "\tpatient: " << indexOfTopLevelItem(item->parent());
    qDebug() << "\texamination: " << currentIndex().row();
  }
}

void PatientWidget::buildTree(const QList<PatientData>& patients)
{
  for (auto& patient : patients) {
    auto top_item = addPatient(patient);
    for (auto& exam : Database::selectExamination(patient.id)) {
      addExamination(top_item, exam);
    }
  }
}

void PatientWidget::onSaveExamination(ExaminationData data)
{
  ExaminationData saved;
  data.patient_id = currentItem()->data(0, ID).toInt();
  Database::insertExamination(data, saved);
  addExamination(currentItem(), saved);
}

void PatientWidget::removeExamination()
{
  auto item = currentItem();
  if (!item) {
    qDebug() << "No current item selected.";
    return;
  }
  if (!PatientWidgetItem::isExamination(item)) {
    QMessageBox::information(this, "Usun badanie", "W celu usuniecia badania zaznacz badanie.");
    return;
  }
  int id = PatientWidgetItem::getId(item);
  Database::deleteExamination(id);
  item->parent()->removeChild(item);
}

void PatientWidget::showScan()
{
  auto item = currentItem();
  if (!item) {
    qDebug() << "[WARNING] No item currently selected.";
    return;
  }
  if (PatientWidgetItem::isExamination(item)) {
    QMessageBox::information(this, "Pokaz skan", "W celu otwarcia skanu zaznacz badanie.");
    return;
  }
  int id = PatientWidgetItem::getId(item);
  ExaminationData out;
  Database::selectExamination(id, out);
  emit openScan(out.name);
}

void PatientWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
  qDebug() << "Double clicked: " << item->text(0) << " (" << PatientWidgetItem::getId(item) << ")";
}

QTreeWidgetItem* PatientWidget::addPatient(PatientData data)
{
  auto item = PatientWidgetItem::createPatientItem(data.id, data.prepareLabel());
  item->setIcon(0, QIcon("icon/broken8.png"));
  addTopLevelItem(item);
  return item;
}

QTreeWidgetItem* PatientWidget::addExamination(QTreeWidgetItem* parent, ExaminationData data)
{
  auto item = PatientWidgetItem::createExamItem(data.id, data.prepareLabel());
  item->setIcon(0, QIcon("icon/stethoscope1.png"));
  parent->addChild(item);
  return item;
}

void PatientWidget::onDeletePatient()
{
  qDebug() << "To delete patient with ID: " << currentItem()->data(0, ID);
  removePatient();
}

void PatientWidget::removeCurrentItem()
{
  QTreeWidgetItem* item = currentItem();
  if (item != nullptr) {
    qDebug() << "Remove current item: " << item->text(0) 
             << " (" << (PatientWidgetItem::isPatient(item) ? "PATIENT" : "EXAMINATION") 
             << ":" << PatientWidgetItem::getId(item) << ")";
    if (PatientWidgetItem::isPatient(item)) {
      //Database::deletePatient(currentItem()->data(0, ID).toInt());
      auto item = takeTopLevelItem(currentIndex().row());
      delete item;
    }
    else if (PatientWidgetItem::isExamination(item)) {

    }
    else {
      qDebug() << "[ERROR]: Wrong type of item in PatientWidget with name: " << item->text(0) << ".";
    }
  }
}

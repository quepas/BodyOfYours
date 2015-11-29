#include "PatientWidget.h"
#include "Database.h"
#include "PatientWidgetItem.h"

#include <QTreeWidgetItem>

#include <QDebug>
#include <QMessageBox>
#include <QAction>
#include <QPushButton>

PatientWidget::PatientWidget(FormViewer* form_viewer, const QList<PatientData>& patients, QWidget* parent /*= 0*/)
  : QTreeWidget(parent), form_viewer_(form_viewer)
{
  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  setHeaderLabels(QStringList(("Pacjent")));
  buildTree(patients);

  // init signal/slots
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
  connect(this, &PatientWidget::currentItemChanged, [=](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
    qDebug() << "PatientWidget::currentItemChanged()";
    if (current) {
      QString text = current->text(0);
      int id = PatientWidgetItem::getId(current);
      qDebug() << "Currently seleted ID: " << id;
      if (PatientWidgetItem::isPatient(current)) {
        PatientData patient;
        Database::selectPatient(id, patient);
        form_viewer_->showPatient(patient);
      }
      else if (PatientWidgetItem::isExamination(current)) {
        ExaminationData exam;
        Database::selectExamination(id, exam);
        form_viewer_->ShowExamination(exam);
      }
      emit showTabWithIndex(0);
    }
  });
}

PatientWidget::~PatientWidget()
{

}

void PatientWidget::onSavePatient(PatientData data)
{
  PatientData saved;
  Database::insertPatient(data, saved);
  addPatient(saved);
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

void PatientWidget::showScan()
{
  auto item = currentItem();
  if (!item) {
    qDebug() << "[WARNING] No item currently selected.";
    return;
  }
  if (!PatientWidgetItem::isExamination(item)) {
    QMessageBox::information(this, "Pokaz skan", "W celu otwarcia skanu zaznacz badanie.");
    return;
  }
  int id = PatientWidgetItem::getId(item);
  ExaminationData out;
  Database::selectExamination(id, out);
  emit openScan("data/" + out.scan_name);
}

void PatientWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
  qDebug() << "[INFO] Double click on: " << item->text(0) << " (" << PatientWidgetItem::getId(item) << ")";
  if (PatientWidgetItem::isExamination(item)) {
    ExaminationData data;
    Database::selectExamination(PatientWidgetItem::getId(item), data);
    qDebug() << "[INFO] Opening 3d mesh: " << data.scan_name;
    emit openScan("data/" + data.scan_name);
    emit showTabWithIndex(1);
  }
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
}

void PatientWidget::removeCurrentItem()
{
  QTreeWidgetItem* item = currentItem();
  if (item != nullptr) {
    int id = PatientWidgetItem::getId(item);
    qDebug() << "Remove current item: " << item->text(0) 
             << " (" << (PatientWidgetItem::isPatient(item) ? "PATIENT" : "EXAMINATION") 
             << ":" << id << ")";
    if (PatientWidgetItem::isPatient(item)) {
      // Remove children
      for (int i = 0; i < item->childCount(); ++i) {
        delete item->child(i);
      }
      // Remove parent
      delete item;
      if (!Database::deletePatient(id)) {
        qDebug() << "[ERROR] Couldn't remove patient from DB with id: " << id;
      }
    }
    else if (PatientWidgetItem::isExamination(item)) {
      delete item;
      if (!Database::deleteExamination(id)) {
        qDebug() << "[ERROR] Couldn't remove examination from DB with id: " << id;
      }
    }
    else {
      qDebug() << "[WARNING] Wrong type of item in PatientWidget with name: " << item->text(0);
    }
  }
}

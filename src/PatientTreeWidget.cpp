#include "PatientTreeWidget.h"
#include "Database.h"
#include "PatientTreeItem.h"

#include <QTreeWidgetItem>

#include <QDebug>
#include <QMessageBox>
#include <QAction>
#include <QPushButton>

PatientTreeWidget::PatientTreeWidget(FormViewer* form_viewer, const QList<PatientData>& patients, QWidget* parent /*= 0*/)
  : QTreeWidget(parent), form_viewer_(form_viewer)
{
  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  setHeaderLabels(QStringList(("Pacjent")));
  buildTree(patients);

  connect(form_viewer->patient_form(), SIGNAL(savePatient(PatientData)), this, SLOT(onSavePatient(PatientData)));
  connect(form_viewer->examination_form(), SIGNAL(saveExam(ExaminationData)), this, SLOT(onSaveExamination(ExaminationData)));
  // init signal/slots
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
  connect(this, &PatientTreeWidget::currentItemChanged, [=](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
    qDebug() << "PatientWidget::currentItemChanged()";
    if (current) {
      QString text = current->text(0);
      int id = PatientTreeItem::getId(current);
      qDebug() << "Currently seleted ID: " << id;
      if (PatientTreeItem::isPatient(current)) {
        PatientData patient;
        Database::selectPatient(id, patient);
        form_viewer_->showPatient(patient);
      }
      else if (PatientTreeItem::isExamination(current)) {
        ExaminationData exam;
        Database::selectExamination(id, exam);
        form_viewer_->showExamination(exam);
      }
      emit showTabWithIndex(0);
    }
  });
}

PatientTreeWidget::~PatientTreeWidget()
{

}

void PatientTreeWidget::onSavePatient(PatientData data)
{
  PatientData saved;
  Database::insertPatient(data, saved);
  addPatient(saved);
}

void PatientTreeWidget::buildTree(const QList<PatientData>& patients)
{
  for (auto& patient : patients) {
    auto top_item = addPatient(patient);
    for (auto& exam : Database::selectExamination(patient.id)) {
      addExamination(top_item, exam);
    }
  }
}

void PatientTreeWidget::onSaveExamination(ExaminationData data)
{
  ExaminationData saved;
  data.patient_id = currentItem()->data(0, ID).toInt();
  Database::insertExamination(data, saved);
  addExamination(currentItem(), saved);
}

void PatientTreeWidget::showScan()
{
  auto item = currentItem();
  if (!item) {
    qDebug() << "[WARNING] No item currently selected.";
    return;
  }
  if (!PatientTreeItem::isExamination(item)) {
    QMessageBox::information(this, "Pokaz skan", "W celu otwarcia skanu zaznacz badanie.");
    return;
  }
  int id = PatientTreeItem::getId(item);
  ExaminationData out;
  Database::selectExamination(id, out);
  emit openScan("data/" + out.scan_name);
}

void PatientTreeWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
  qDebug() << "[INFO] Double click on: " << item->text(0) << " (" << PatientTreeItem::getId(item) << ")";
  if (PatientTreeItem::isExamination(item)) {
    ExaminationData data;
    Database::selectExamination(PatientTreeItem::getId(item), data);
    qDebug() << "[INFO] Opening 3d mesh: " << data.scan_name;
    emit openScan("data/" + data.scan_name);
    emit showTabWithIndex(1);
  }
}

QTreeWidgetItem* PatientTreeWidget::addPatient(PatientData data)
{
  auto item = PatientTreeItem::createPatientItem(data.id, data.prepareLabel());
  item->setIcon(0, QIcon("icon/broken8.png"));
  addTopLevelItem(item);
  return item;
}

QTreeWidgetItem* PatientTreeWidget::addExamination(QTreeWidgetItem* parent, ExaminationData data)
{
  auto item = PatientTreeItem::createExamItem(data.id, data.prepareLabel());
  item->setIcon(0, QIcon("icon/stethoscope1.png"));
  parent->addChild(item);
  return item;
}

void PatientTreeWidget::onDeletePatient()
{
  qDebug() << "To delete patient with ID: " << currentItem()->data(0, ID);
}

void PatientTreeWidget::removeCurrentItem()
{
  QTreeWidgetItem* item = currentItem();
  if (item != nullptr) {
    int id = PatientTreeItem::getId(item);
    qDebug() << "Remove current item: " << item->text(0) 
             << " (" << (PatientTreeItem::isPatient(item) ? "PATIENT" : "EXAMINATION") 
             << ":" << id << ")";
    if (PatientTreeItem::isPatient(item)) {
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
    else if (PatientTreeItem::isExamination(item)) {
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

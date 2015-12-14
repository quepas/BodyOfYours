#include "PatientTreeWidget.h"
#include "Database.h"
#include "PatientTreeItem.h"
#include "PatientForm.h"
#include "ExaminationForm.h"

#include <QTreeWidgetItem>

#include <QDebug>
#include <QMessageBox>
#include <QAction>
#include <QPushButton>

PatientTreeWidget::PatientTreeWidget(QSqlTableModel* patient_model, QSqlTableModel* exam_model, StackedFormWidget* form_widget, const QList<PatientData>& patients, QWidget* parent /*= 0*/) : QTreeWidget(parent), form_widget_(form_widget), patient_model_(patient_model), exam_model_(exam_model)
{
  // setup Scan model
  scan_model_ = new QSqlTableModel();
  scan_model_->setTable("scan");
  scan_model_->select();

  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  setHeaderLabels(QStringList(("Pacjent")));
  connect(patient_model_, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  connect(exam_model_, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  connect(scan_model_, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  buildTreeFromModel(patient_model_, exam_model_);
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
        form_widget->switchTo(StackedFormWidget::PATIENT_FORM, id);
      }
      else if (PatientTreeItem::isExamination(current)) {
        ExaminationData exam;
        Database::selectExamination(id, exam);
        form_widget->switchTo(StackedFormWidget::EXAMINATION_FORM, id);
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

void PatientTreeWidget::onModifyPatient(PatientData data)
{
  Database::updatePatientOnly(data);
  modifyPatient(data);
}

void PatientTreeWidget::buildTreeFromModel(QSqlTableModel* patient_model, QSqlTableModel* exam_model)
{
  for (int i = 0; i < patient_model->rowCount(); ++i) {
    QSqlRecord record = patient_model->record(i);
    int id = record.value("id").toInt();
    auto item = PatientTreeItem::createPatientItem(id,
      record.value("name").toString() + " " + record.value("surname").toString() +
      "(" + record.value("pesel").toString() + ")");
    item->setIcon(0, QIcon("icon/broken8.png"));
    addTopLevelItem(item);
    // insert patient's examinations
    for (int j = 0; j < exam_model->rowCount(); ++j) {
      QSqlRecord exam = exam_model->record(j);
      int exam_fk_id = exam.value("patient_id").toInt();
      int exam_id = exam.value("id").toInt();
      if (exam_fk_id == id) {
        auto exam_item = PatientTreeItem::createExamItem(exam_id, exam.value("name").toString());
        exam_item->setIcon(0, QIcon("icon/stethoscope1.png"));
        item->addChild(exam_item);
        // insert exam's scans
        for (int k = 0; k < scan_model_->rowCount(); ++k) {
          QSqlRecord scan = scan_model_->record(k);
          int scan_exam_fk_id = scan.value("exam_id").toInt();
          if (scan_exam_fk_id == exam_id) {
            auto scan_item = PatientTreeItem::createScanItem(scan.value("id").toInt(), scan.value("filename").toString());
            scan_item->setIcon(0, QIcon("icon/radiography.png"));
            exam_item->addChild(scan_item);
          }
        }
      }
    }
  }
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

QTreeWidgetItem* PatientTreeWidget::modifyPatient(PatientData data)
{
  for (int i = 0; i < topLevelItemCount(); ++i) {
    auto item = topLevelItem(i);
    if (PatientTreeItem::isPatient(item) && PatientTreeItem::getId(item) == data.id) {
      item->setText(0, data.prepareLabel());
      return item;
    }
  }
  return nullptr;
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

void PatientTreeWidget::onDataChanged()
{
  qDebug() << "onDataChanged()";
  clear();
  buildTreeFromModel(patient_model_, exam_model_);
}

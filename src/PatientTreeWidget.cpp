#include "PatientTreeWidget.h"
#include "Database.h"
#include "PatientTreeItem.h"
#include "PatientForm.h"
#include "ExaminationForm.h"
#include "ScanViewer.h"
#include "ModelHelper.h"

#include <QTreeWidgetItem>

#include <QDebug>
#include <QMessageBox>
#include <QAction>
#include <QPushButton>

PatientTreeWidget::PatientTreeWidget(SQLTableModelHandler handler, StackedFormWidget* form_widget, const QList<PatientData>& patients, QWidget* parent /*= 0*/) : QTreeWidget(parent), form_widget_(form_widget), handler_(handler)
{
  setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  setHeaderLabels(QStringList(("Pacjent")));
  connect(handler.patient, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  connect(handler.examination, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  connect(handler.scan, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  // TODO: do we need to update scan_diff model?
  connect(handler.scan_diff, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(onDataChanged()));
  buildTreeFromModel(handler_);
  // init signal/slots
  connect(this, &PatientTreeWidget::itemSelectionChanged, [=]{
    auto item = currentItem();
    if (!item) return;
    int itemID = PatientTreeItem::getId(item);
    if (PatientTreeItem::isPatient(item)) {
      form_widget->switchTo(StackedFormWidget::PATIENT_FORM, itemID);
    }
    else if (PatientTreeItem::isExamination(item)) {
      form_widget->switchTo(StackedFormWidget::EXAMINATION_FORM, itemID);
    }
    else if (PatientTreeItem::isScan(item)) {
      form_widget->switchTo(StackedFormWidget::SCAN_FORM, itemID);
#ifndef _DEBUG
      int scanID = PatientTreeItem::getId(item);
      emit displayScan(ScanViewer::MINI_VIEWER, scanID);
#endif
    }
    emit showTabWithIndex(0);
  });
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
}

PatientTreeWidget::~PatientTreeWidget()
{

}

void PatientTreeWidget::buildTreeFromModel(SQLTableModelHandler handler)
{
  for (int i = 0; i < handler.patient->rowCount(); ++i) {
    QSqlRecord record = handler.patient->record(i);
    if (!record.isNull("id")) {
      int id = record.value("id").toInt();
      auto item = PatientTreeItem::createPatientItem(id,
        record.value("name").toString() + " " + record.value("surname").toString() +
        " (" + record.value("pesel").toString() + ")");
      item->setIcon(0, QIcon("icon/broken8.png"));
      addTopLevelItem(item);
      // insert patient's examinations
      for (int j = 0; j < handler.examination->rowCount(); ++j) {
        QSqlRecord exam = handler.examination->record(j);
        if (!exam.isNull("id")) {
          int exam_fk_id = exam.value("patient_id").toInt();
          int exam_id = exam.value("id").toInt();
          if (exam_fk_id == id) {
            auto exam_item = PatientTreeItem::createExamItem(exam_id, exam.value("name").toString());
            exam_item->setIcon(0, QIcon("icon/stethoscope1.png"));
            item->addChild(exam_item);
            // insert exam's scans
            for (int k = 0; k < handler.scan->rowCount(); ++k) {
              QSqlRecord scan = handler.scan->record(k);
              if (!scan.isNull("id")) {
                int scan_exam_fk_id = scan.value("exam_id").toInt();
                if (scan_exam_fk_id == exam_id) {
                  QString scanName = scan.value("name").toString();
                  QString scanFileName = scan.value("filename").toString();
                  auto scan_item = PatientTreeItem::createScanItem(scan.value("id").toInt(), !scanName.isEmpty() ? scanName : scanFileName);
                  scan_item->setIcon(0, QIcon("icon/radiography.png"));
                  exam_item->addChild(scan_item);
                }
              }
            }
          }
        }
      }
    }
  }
}

void PatientTreeWidget::showCurrentScan()
{
  auto item = currentItem();
  if (PatientTreeItem::isScan(item)) {
    emit displayScan(ScanViewer::ID::FULL_VIEWER, PatientTreeItem::getId(item));
  }
}

void PatientTreeWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
  if (PatientTreeItem::isScan(item)) {
    int scanID = PatientTreeItem::getId(item);
    emit displayScan(ScanViewer::ID::FULL_VIEWER, scanID);
  }
}

void PatientTreeWidget::removeCurrentItem()
{
  QTreeWidgetItem* item = currentItem();
  if (item != nullptr) {
    int id = PatientTreeItem::getId(item);
    switch (item->type()) {
    case PATIENT:
      ModelHelper::deletePatient(id, handler_);
      break;
    case EXAM:
      ModelHelper::deleteExaminations(id, handler_);
      break;
    case SCAN:
      ModelHelper::deleteScans(id, handler_);
      break;
    }
  }
}

void PatientTreeWidget::onDataChanged()
{
  saveExpanded();
  qDebug() << "---> Rebuild patient tree";
  clear();
  buildTreeFromModel(handler_);
  restorExpanded();

}

void PatientTreeWidget::saveExpanded()
{
  patients_exp_.clear();
  exam_exp_.clear();

  for (int i = 0; i < topLevelItemCount(); ++i) {
    auto topItem = topLevelItem(i);
    if (PatientTreeItem::isPatient(topItem)) {
      if (topItem->isExpanded()) patients_exp_.append(PatientTreeItem::getId(topItem));
      for (int k = 0; k < topItem->childCount(); ++k) {
        auto child = topItem->child(k);
        if (PatientTreeItem::isExamination(child)) {
          if (child->isExpanded()) exam_exp_.append(PatientTreeItem::getId(child));
        }
      }
    }
  }
}

void PatientTreeWidget::restorExpanded()
{
  for (int i = 0; i < topLevelItemCount(); ++i) {
    auto topItem = topLevelItem(i);
    if (PatientTreeItem::isPatient(topItem)) {
      int id = PatientTreeItem::getId(topItem);
      if (patients_exp_.contains(id)) topItem->setExpanded(true);
      for (int k = 0; k < topItem->childCount(); ++k) {
        auto child = topItem->child(k);
        if (PatientTreeItem::isExamination(child)) {
          int childID = PatientTreeItem::getId(child);
          if (exam_exp_.contains(childID)) child->setExpanded(true);
        }
      }
    }
  }
}

#pragma once

#include "ActionHub.h"
#include "Action.h"

#include "StackedFormWidget.h"
#include "ScanViewer.h"
#include "PatientTreeWidget.h"
#include "Scanner.h"

CREATE_ACTION(ActionAddNewPatient, {
  widget_->switchTo(StackedFormWidget::PATIENT_FORM);
}, StackedFormWidget* widget_)

CREATE_ACTION(ActionAddNewExamination, {
  widget_->switchTo(StackedFormWidget::EXAMINATION_FORM);
}, StackedFormWidget* widget_)

CREATE_ACTION(ActionAddNewScan, {
  widget_->switchTo(StackedFormWidget::SCAN_FORM);
} , StackedFormWidget* widget_)

CREATE_ACTION(ActionDeleteCurrentItem, {
  widget->removeCurrentItem();
}, PatientTreeWidget* widget)

CREATE_ACTION(ActionShowScanMesh, {
  widget->showCurrentScan();
}, PatientTreeWidget* widget)

CREATE_ACTION(ActionStartReconstruction, {
  scanner->startReconstruction();
}, Scanner* scanner)

CREATE_ACTION(ActionStopReconstruction, {
  scanner->stopReconstruction();
}, Scanner* scanner)

CREATE_ACTION(ActionMeshViewerClear, {
  viewer->clearDisplay();
}, ScanViewer* viewer);

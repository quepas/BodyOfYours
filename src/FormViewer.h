#pragma once

#include "PatientData.h"
#include "ExaminationData.h"
#include <QStackedWidget>

#include "PForm.h"
#include "EForm.h"

class FormViewer : public QStackedWidget
{
public:
  FormViewer(QWidget* parent = nullptr);
  ~FormViewer();

public slots:
  void showPatient(PatientData data);
  void showExamination(ExaminationData data);
  void newPatient();
  void newExamination();

private:
  PForm* patient_form_;
  EForm* exam_form_;

};
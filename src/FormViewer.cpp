#include "FormViewer.h"

FormViewer::FormViewer(QWidget* parent /*= nullptr*/)
  : QStackedWidget(parent)
{
  patient_form_ = new PatientForm(this);
  exam_form_ = new ExaminationForm(this);
  addWidget(patient_form_);
  addWidget(exam_form_);
}

FormViewer::~FormViewer()
{
  delete patient_form_;
  delete exam_form_;
}

void FormViewer::showPatient(PatientData data)
{
  patient_form_->clear();
  exam_form_->clear();
  setCurrentIndex(0);
  patient_form_->setData(data);
}

void FormViewer::ShowExamination(ExaminationData data)
{
  patient_form_->clear();
  exam_form_->clear();
  setCurrentIndex(1);
  exam_form_->setData(data);
}

void FormViewer::newPatient()
{
  setCurrentIndex(0);
}

void FormViewer::newExamination()
{
  setCurrentIndex(1);
}

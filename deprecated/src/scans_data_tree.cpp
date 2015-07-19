#include "scans_data_tree.h"
#include "scaninfodialog.h"
#include "resources.h"
#include "patientinfodialog.h"

#include <QAction>
#include <QDebug>

ScansDataTree::ScansDataTree(QWidget* parent /*= 0*/)
  : QTreeView(parent),
    model_(new PatientTreeModel("./data/patients/"))
{
  setModel(model_);
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(CustomContextMenuSlot(const QPoint&)));
  InitPatientContextMenu();
  InitScanContextMenu();
}

ScansDataTree::~ScansDataTree()
{
  delete model_;
}

void ScansDataTree::CustomContextMenuSlot(const QPoint& point)
{
  QModelIndex index = indexAt(point);
  if (index.isValid() && index.parent() == model_->invisibleRootItem()->index()) {
    patient_context_menu_->exec(mapToGlobal(point));
  } else {
    scan_context_menu_->exec(mapToGlobal(point));
  }
}

void ScansDataTree::InitPatientContextMenu()
{
  patient_context_menu_ = new QMenu(this);
  QAction* remove_patient = new QAction(QIcon(Resources::ICON_REMOVE), "Remove patient", nullptr);
  QAction* modify_patient = new QAction(QIcon(Resources::ICON_MODIFY), "Update patient", nullptr);
  scan_patient_ = new QAction(QIcon(Resources::ICON_CREATE), "Make new scan", nullptr);
  scan_patient_->setEnabled(false);
  patient_context_menu_->addAction(remove_patient);
  patient_context_menu_->addAction(modify_patient);
  patient_context_menu_->addAction(scan_patient_);
  connect(remove_patient, SIGNAL(triggered()), this, SLOT(RemoveSelectedSlot()));
  connect(scan_patient_, SIGNAL(triggered()), this, SLOT(ScanPatient()));
  connect(modify_patient, SIGNAL(triggered()), this, SLOT(ModifyPatient()));
}

void ScansDataTree::InitScanContextMenu()
{
  scan_context_menu_ = new QMenu(this);
  QAction* remove_scan = new QAction(QIcon(Resources::ICON_REMOVE), "Remove scan", nullptr);
  QAction* modify_scan = new QAction(QIcon(Resources::ICON_MODIFY), "Modify scan info", nullptr);
  QAction* visualize_scan = new QAction(QIcon(Resources::ICON_SCAN), "Visualize scan", nullptr);
  QAction* smooth_scan = new QAction(QIcon(Resources::ICON_MODIFY), "Smooth scan", nullptr);
  scan_context_menu_->addAction(remove_scan);
  scan_context_menu_->addAction(modify_scan);
  scan_context_menu_->addAction(visualize_scan);
  scan_context_menu_->addAction(smooth_scan);
  connect(remove_scan, SIGNAL(triggered()), this, SLOT(RemoveScanSlot()));
  connect(modify_scan, SIGNAL(triggered()), this, SLOT(ModifyScanSlot()));
  connect(visualize_scan, SIGNAL(triggered()), this, SLOT(VisualizeScanSlot()));
  connect(smooth_scan, SIGNAL(triggered()), this, SLOT(SmoothScanSlot()));
}

bool ScansDataTree::RemoveSelected()
{
  QModelIndex index = currentIndex();
  if (index.isValid()) {
    QString patient_id = model_->patients()[index.row()].id();
    model_->Delete(patient_id);
  }
  return false;
}

void ScansDataTree::RemoveSelectedSlot()
{
  RemoveSelected();
}

void ScansDataTree::SetScanActionEnable(bool enabled)
{
  scan_patient_->setEnabled(enabled);
}

void ScansDataTree::ScanPatient()
{
  QModelIndex index = currentIndex();
  if (index.isValid()) {
    scanning_window_->Show(model_->patients()[index.row()]);
    scanning_window_->StartGrabbingData();
  }
}

void ScansDataTree::ModifyPatient()
{
  QModelIndex index = currentIndex();
  if (index.isValid()) {
    Patient patient = model_->patients()[index.row()];
    PatientInfoDialog* dialog = new PatientInfoDialog(patient);
    connect(dialog, SIGNAL(UpdatePatientSignal(Patient)), this, SLOT(UpdatePatientSlot(Patient)));
    dialog->show();
  }
}

void ScansDataTree::UpdatePatientSlot(Patient patient)
{
  model_->Update(patient);
}

void ScansDataTree::RemoveScanSlot()
{
  QModelIndex index = currentIndex();
  if (index.isValid()) {
    Patient patient = model_->patients()[index.parent().row()];
    QString filename = patient.scans()[index.row()].filename();
    QVector<Scan> scans = patient.scans();
    scans.remove(index.row());
    patient.set_scans(scans);
    // remove scan file
    QDir path("./data/patients/" + patient.id() + "/scans/");
    path.remove(filename);
    // update patient data
    model_->Update(patient);
  }
}

void ScansDataTree::VisualizeScanSlot()
{
  QModelIndex index = currentIndex();
  if (index.isValid()) {
    Patient patient = model_->patients()[index.parent().row()];
    Scan scan = patient.scans()[index.row()];
    QString scan_full_path = "./data/patients/" + patient.id() + "/scans/" + scan.filename();
    emit VisualizeScanSignal(scan_full_path);
  }
}

void ScansDataTree::ModifyScanSlot()
{
  QModelIndex index = currentIndex();
  if (index.isValid()) {
    Patient patient = model_->patients()[index.parent().row()];
    Scan scan = patient.scans()[index.row()];
    ScanInfoDialog* dialog = new ScanInfoDialog(nullptr, patient, scan);
    connect(dialog, SIGNAL(UpdatePatientSignal(Patient)), this, SLOT(UpdatePatientSlot(Patient)));
    dialog->show();
  }
}

void ScansDataTree::SmoothScanSlot()
{
  qDebug() << "Smoothing";
}

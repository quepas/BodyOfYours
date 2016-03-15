#pragma once

#include <QSqlTableModel>

struct SQLTableModelHandler
{
  SQLTableModelHandler(QSqlTableModel* _patient, QSqlTableModel* _examination, QSqlTableModel* _scan, QSqlTableModel* _scan_diff)
    : patient(_patient), examination(_examination), scan(_scan), scan_diff(_scan_diff)
  {};

  QSqlTableModel* patient;
  QSqlTableModel* examination;
  QSqlTableModel* scan_diff;
  QSqlTableModel* scan;
};

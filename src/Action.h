#pragma once

#include <QAction>
#include <QString>

#define CREATE_ACTION(name, code, ...) \
  class name : public Action { \
    public: \
      name(QObject* parent, __VA_ARGS__) : Action(parent) { \
        connect(this, &QAction::triggered, [=] { \
          code \
        }); \
      } \
      QString type() { return #name; } \
      static QString TYPE() { return #name; } \
  };

class Action : public QAction
{
  Q_OBJECT
public:
  Action(QObject* parent) : QAction(parent) {}
  virtual ~Action() {}
  virtual QString type() = 0;
};

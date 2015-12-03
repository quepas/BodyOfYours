#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

class Action;

class ActionHub
{
private:
  ActionHub() {};
  ActionHub(const ActionHub&);
  ActionHub& operator=(const ActionHub&);
  ~ActionHub();

  QMap<QString, Action*> actions_;

public:
  static ActionHub& instance() {
    static ActionHub instance;
    return instance;
  }

  static void addAction(Action* action);
  static void trigger(QString id);
  void registerAction(Action* action);
  void execute(QString id);
  Action* action(QString id);
};

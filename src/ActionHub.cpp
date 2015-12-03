#include "ActionHub.h"

#include "Action.h"
#include <QDebug>

ActionHub::~ActionHub()
{
  for (auto& action : actions_) {
    delete action;
  }
}

void ActionHub::addAction(Action* action)
{
  instance().registerAction(action);
}

void ActionHub::trigger(QString id)
{
  instance().execute(id);
}

Action* ActionHub::action(QString id)
{
  if (!actions_.contains(id)) {
    qDebug() << "[WARNING] ActionHub doesn't contain " << id;
    return nullptr;
  }
  return actions_[id];
}

void ActionHub::execute(QString id)
{
  if (!actions_.contains(id)) {
    qDebug() << "[WARNING] ActionHub doesn't containt action " << id;
    return;
  }
  qDebug() << "[INFO] Executing " << id;
  actions_[id]->trigger();
}

void ActionHub::registerAction(Action* action)
{
  actions_.insert(action->type(), action);
}

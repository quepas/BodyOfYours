#pragma once

#include <reconstructmesdk/reme.h>
#include <QThread>

class RemeSensorCheck : public QThread
{
public:
  RemeSensorCheck(reme_context_t context, const char* driver);

protected:
  void run();

private:
  reme_context_t context_;
  const char* driver_;
};

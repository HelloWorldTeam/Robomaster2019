//
// Created by wx on 18-4-28.
//

#include "communicator.h"
#include "setting.h"

#ifndef ROBOMASTERRUNEDETECTOR_REMOTECONTROL_H
#define ROBOMASTERRUNEDETECTOR_REMOTECONTROL_H
class RemoteControl {
 public:
  RemoteControl(Setting* _setting, COMMUNICATOR& tmp)
      : setting(_setting), communicator(tmp) {}
  Setting* setting;

 private:
  COMMUNICATOR& communicator;

 public:
  void Receiver();
};

#endif  // ROBOMASTERRUNEDETECTOR_REMOTECONTROL_H

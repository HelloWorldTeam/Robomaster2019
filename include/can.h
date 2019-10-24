//
// Created by liming on 4/28/18.
// ReCreated by mfh on 19.03.05.
//

#ifndef __CAN_H__
#define __CAN_H__
#include "communicator.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

class CAN : public COMMUNICATOR {
 public:
  CAN(const char *pathname);
  ~CAN() { close(port_handle); }
  virtual int receive(char *buffer);
  virtual int send(double X, double Y, double Z, int type);
  virtual int senderr();

 private:
  int port_handle;
  int send(double yaw, double pitch);
  int send(double X, double Y, double Z);
  CAN() {}
};

#endif  //__CAN_H__

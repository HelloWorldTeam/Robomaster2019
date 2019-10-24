//
// Created by xiangpu on 19-1-13.
// ReCreated by flyingtiger on 19-03-04.
//
#include "communicator.h"
#ifndef ROBOMASTERSHAOBING_NEW_UART_H
#define ROBOMASTERSHAOBING_NEW_UART_H

class UART : public COMMUNICATOR {
 public:
  UART(const char *pathname, const char *pathname_bkp);
  ~UART();
  virtual int receive(char *buffer);
  virtual int send(double X, double Y, double Z, int type);
  virtual int senderr() { return 0; }

 private:
  int fd;
  int uart_set(int baude, int c_flow, int bits, char parity, int stop);
  void init(const char *pathname, const char *pathname_bkp);
  ssize_t safe_write(const void *vptr, size_t n);
  ssize_t safe_read(void *vptr, size_t n);
  int uart_write(const char *w_buf, size_t len);
  int uart_read(char *r_buf, size_t len);
  UART() : fd(-1) {}
};

#endif  // ROBOMASTERSHAOBING_NEW_UART_H

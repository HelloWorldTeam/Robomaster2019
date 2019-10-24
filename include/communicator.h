//
// Created by flyingtiger on 19-03-04.
//

#ifndef ROBOMASTERSHAOBING_COMMUNICATOR_H
#define ROBOMASTERSHAOBING_COMMUNICATOR_H
#include <thread>
class COMMUNICATOR {
 public:
  COMMUNICATOR() {}
  virtual ~COMMUNICATOR() {}
  virtual int receive(char* buffer) { 
    buffer[2] = 1;//= !buffer[2];
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
  }
  virtual int send(double X, double Y, double Z, int type) { return 0; }
  virtual int senderr() { return 0; }

 private:
};
#endif  // ROBOMASTERSHAOBING_COMMUNICATOR_H

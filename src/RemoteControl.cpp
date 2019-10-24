//
// Created by wx on 18-4-28.
// Modified by flyingtiger on 19-03-04.
//

#include "RemoteControl.h"
#include <iostream>
#include <thread>
#include "uart.h"
extern bool RUNNING;

void RemoteControl::Receiver() {
  char buffer[8] = {0};
  while (RUNNING) {
    /// receive
    int num_bytes = communicator.receive(buffer);
    if (num_bytes <= 0) continue;
    char cmd1 = buffer[0];
    char cmd2 = buffer[1];

    switch (cmd1) {
      case 0: {
        setting->mode = 0;
        printf("Stop auto-shoooot\n");
        break;
      }
      case 1: {
        setting->mode = 1; //auto-shoot
        setting->color= 1;
        printf("Start auto-shoooot red\n");
        break;
      }
      case 2: {
        setting->mode = 1; //auto-shoot
        setting->color= 2;
        printf("Start auto-shoooot blue\n");
        break;
      }
      case 0x7f: {
        printf("-------------------------------------shutdown!!!\n");
        int retval = system("sudo shutdown now");
      }
      default:
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

//
// Created by wx on 18-4-28.
//

#ifndef ROBOMASTERRUNEDETECTOR_SETTING_H
#define ROBOMASTERRUNEDETECTOR_SETTING_H
class Setting {
 public:
  Setting() { mode = 0; color = 0;}

 public:
  int mode;  // 0：not working； 1：auto-shooooot
  int color; // 0: all 1: red 2: blue
};

#endif  // ROBOMASTERRUNEDETECTOR_SETTING_H

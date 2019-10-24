//
// Created by liming on 1/2/18.
//

#ifndef MARKERSENSOR_TIMER_H
#define MARKERSENSOR_TIMER_H

#include <sys/time.h>

#define TIME_BEGIN()  \
  {                   \
    timeval t_b, t_e; \
    gettimeofday(&t_b, 0);
#define TIME_END(TAG)                                                        \
  gettimeofday(&t_e, 0);                                                     \
  printf(                                                                    \
      "=== %s time: %08.3f ms\n", TAG,                                       \
      (t_e.tv_sec - t_b.tv_sec) * 1e3 + (t_e.tv_usec - t_b.tv_usec) * 1e-3); \
  }

#endif  // MARKERSENSOR_TIMER_H

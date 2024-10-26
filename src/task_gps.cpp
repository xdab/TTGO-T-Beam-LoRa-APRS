#include "task_gps.h"
#include <gps.h>

void task_gps(void *parameter)
{
  gps.initialize();
  for (;;)
    gps._read_serial();
}

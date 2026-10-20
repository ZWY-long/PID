#define PORTABLE_PID_IMPLEMENTATION
#include "../通用比例积分微分.h"
void main(void) { PortablePid p; PortablePidConfig c; c=portable_pid_heater_config(); portable_pid_reset(&p); portable_pid_step(&p,&c,100.0f,25.0f,0.5f); }

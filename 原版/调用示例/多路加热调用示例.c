/* Compile this file alone, or move IMPLEMENTATION to your own main.c. */
#define PORTABLE_PID_IMPLEMENTATION
#include "../通用比例积分微分.h"

#define HEATER_CHANNELS 7
#if defined(__C51__)
#define HEATER_STORAGE xdata
#else
#define HEATER_STORAGE
#endif
static PortablePid HEATER_STORAGE heaters[HEATER_CHANNELS];
static PortablePidConfig heater_cfg;

void heater_init(void)
{
    unsigned char i;
    heater_cfg=portable_pid_heater_config();
    for(i=0;i<HEATER_CHANNELS;i++) portable_pid_reset(&heaters[i]);
}

/* Call on a new valid sample; dt comes from this channel's sample clock.
 * Output is percent. Replace the returned value with your PWM/serial write.
 * A return of -1 asks the caller to FORCE the hardware output OFF.
 */
float heater_on_sample(unsigned char ch, float target, float measured,
                       float dt, unsigned char permit_heat)
{
    if(ch>=HEATER_CHANNELS) return -1.0f;
    if(!permit_heat) {
        portable_pid_reset(&heaters[ch]);
        return 0.0f; /* Write zero immediately, bypassing PID slew limiting. */
    }
    if(!portable_pid_step(&heaters[ch],&heater_cfg,target,measured,dt)) {
        portable_pid_reset(&heaters[ch]);
        return -1.0f;
    }
    return heaters[ch].output;
}

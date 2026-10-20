/* Portable positional PID, extracted 2026-10-19.
 * C89 / C51 friendly. ASCII source, no IO, heap, Flash or global state.
 * Define PORTABLE_PID_IMPLEMENTATION in EXACTLY ONE translation unit.
 * All other translation units include this header without that macro.
 */
#ifndef PORTABLE_PID_H
#define PORTABLE_PID_H

typedef struct {
    float kp, ki, kd;
    float output_min, output_max;
    float p_deadband;       /* input units, strict abs(error) < threshold */
    float d_step_deadband;  /* input change per sample; 0 disables */
    float d_alpha;          /* per-sample low pass coefficient [0,1] */
    float slew_rate;        /* output units/second; 0 disables */
    float default_dt, min_dt, max_dt; /* seconds */
} PortablePidConfig;

typedef struct {
    float integral;         /* accumulated error * seconds (NOT I output) */
    float output, last_out, raw_out;
    float d_filt, last_temp, last_err;
    float actual_temp, target_temp;
    unsigned char have_temp;
} PortablePid;

/* Preset for the validated heater; not universal tuning parameters. */
PortablePidConfig portable_pid_heater_config(void);
int portable_pid_config_valid(const PortablePidConfig *cfg);
void portable_pid_reset(PortablePid *pid);
/* Returns 1 on update, 0 on invalid input/config. Failure leaves state intact.
 * Call only for FRESH measurements. Quantize output at the actuator boundary.
 * dt < min_dt uses default_dt; dt > max_dt is capped (legacy behavior).
 */
int portable_pid_step(PortablePid *pid, const PortablePidConfig *cfg,
                      float target, float actual, float dt);

#endif

#if defined(PORTABLE_PID_IMPLEMENTATION) && !defined(PORTABLE_PID_IMPLEMENTED)
#define PORTABLE_PID_IMPLEMENTED
#include <float.h>

static int portable_pid_finite(float v)
{
    return v == v && v <= FLT_MAX && v >= -FLT_MAX;
}

PortablePidConfig portable_pid_heater_config(void)
{
    PortablePidConfig c;
    c.kp=10.0f; c.ki=0.05f; c.kd=20.0f;
    c.output_min=0.0f; c.output_max=100.0f;
    c.p_deadband=0.3f; c.d_step_deadband=0.1f;
    c.d_alpha=0.2f; c.slew_rate=5.0f;
    c.default_dt=0.5f; c.min_dt=0.05f; c.max_dt=1.0f;
    return c;
}

int portable_pid_config_valid(const PortablePidConfig *c)
{
    if(!c) return 0;
    if(!portable_pid_finite(c->kp) || !portable_pid_finite(c->ki)
       || !portable_pid_finite(c->kd)
       || !portable_pid_finite(c->output_min)
       || !portable_pid_finite(c->output_max)
       || !portable_pid_finite(c->p_deadband)
       || !portable_pid_finite(c->d_step_deadband)
       || !portable_pid_finite(c->d_alpha)
       || !portable_pid_finite(c->slew_rate)
       || !portable_pid_finite(c->default_dt)
       || !portable_pid_finite(c->min_dt)
       || !portable_pid_finite(c->max_dt)) return 0;
    /* Preserve legacy positive-gain, single-direction heating semantics. */
    return c->kp>=0.0f && (c->ki==0.0f || c->ki>0.0001f)
        && c->kd>=0.0f && c->output_min==0.0f
        && c->output_max>c->output_min && c->p_deadband>=0.0f
        && c->d_step_deadband>=0.0f && c->d_alpha>=0.0f
        && c->d_alpha<=1.0f && c->slew_rate>=0.0f
        && c->min_dt>0.0f && c->default_dt>=c->min_dt
        && c->max_dt>=c->default_dt;
}

void portable_pid_reset(PortablePid *p)
{
    if(!p) return;
    p->integral=0.0f; p->output=0.0f; p->last_out=0.0f;
    p->raw_out=0.0f; p->d_filt=0.0f; p->last_temp=0.0f;
    p->last_err=0.0f; p->actual_temp=0.0f; p->target_temp=0.0f;
    p->have_temp=0;
}

int portable_pid_step(PortablePid *p, const PortablePidConfig *c,
                      float target, float actual, float dt)
{
    float err, err_i, out, lo, hi, d_raw, bu, d_men;
    if(!p || !portable_pid_config_valid(c)
       || !portable_pid_finite(target) || !portable_pid_finite(actual)
       || !portable_pid_finite(dt)) return 0;
    if(dt<c->min_dt) dt=c->default_dt;
    if(dt>c->max_dt) dt=c->max_dt;
    p->target_temp=target; p->actual_temp=actual;
    hi=c->output_max; lo=c->output_min;
    err=target-actual;
    err_i=err;
    if(err<c->p_deadband && err>-c->p_deadband) err=0.0f;
    if(!((p->output>=hi && err_i>0.0f)
       || (p->output<=lo && err_i<0.0f))) {
        p->integral+=err_i*dt;
    }
    if(c->ki>0.0001f) {
        if(c->ki*p->integral>hi) p->integral=hi/c->ki;
        if(c->ki*p->integral<lo) p->integral=lo/c->ki;
    } else {
        /* Extension: no hidden integral accumulation when I is disabled. */
        p->integral=0.0f;
    }
    if(!p->have_temp) d_raw=0.0f;
    else d_raw=-(actual-p->last_temp)/dt;
    d_men=c->d_step_deadband/dt;
    if(d_raw<d_men && d_raw>-d_men) d_raw=0.0f;
    p->d_filt+=c->d_alpha*(d_raw-p->d_filt);
    p->last_temp=actual;
    p->have_temp=1;
    out=c->kp*err;
    out+=c->ki*p->integral;
    out+=c->kd*p->d_filt;
    p->last_err=err;
    if(out>hi) out=hi;
    if(out<lo) out=lo;
    p->raw_out=out;
    if(c->slew_rate>0.0f) {
        bu=c->slew_rate*dt;
        if(out-p->last_out>bu) out=p->last_out+bu;
        if(out-p->last_out<-bu) out=p->last_out-bu;
    }
    /* Final clamp also covers a reduced output_max at runtime. */
    if(out>hi) out=hi;
    if(out<lo) out=lo;
    p->output=out;
    p->last_out=out;
    return 1;
}
#endif

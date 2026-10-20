/* 候选参数试跑的流式指标验收工具。只看真实闭环测量，不生成控制输出。
 * 在一个.c定义PORTABLE_TUNE_VALIDATION_IMPLEMENTATION后包含。
 * 无环形波形缓存、无动态内存；异常退出后仍需调用方立即停热。
 */
#ifndef PORTABLE_TUNE_VALIDATION_H
#define PORTABLE_TUNE_VALIDATION_H
#include <limits.h>
#if ULONG_MAX < 4294967295UL
#error Validation_requires_32_bit_unsigned_long
#endif
#define PORTABLE_VALIDATION_IDLE 0
#define PORTABLE_VALIDATION_RUNNING 1
#define PORTABLE_VALIDATION_PASS 2
#define PORTABLE_VALIDATION_FAIL 3
typedef struct {
    float target, tolerance, minimum_inside_fraction;
    float maximum_peak_to_peak, maximum_mean_error, maximum_overshoot;
    unsigned long warmup_ms, window_ms, max_gap_ms;
} PortableTuneValidationConfig;
typedef struct {
    PortableTuneValidationConfig cfg;
    float minimum, maximum, mean, peak, last;
    unsigned long start_ms, sample_ms, tick_ms, count, inside;
    unsigned char state, reason, have_sample;
} PortableTuneValidation;
/* begin在PID用候选参数开始试跑的同一时刻调用；失败输出不归本工具控制。 */
int portable_validation_start(PortableTuneValidation *v,
    const PortableTuneValidationConfig *c, unsigned long now_ms);
int portable_validation_tick(PortableTuneValidation *v, unsigned long now_ms,
    unsigned char permit);
int portable_validation_update(PortableTuneValidation *v, float measurement,
    unsigned long now_ms, unsigned char permit);
#endif

#if defined(PORTABLE_TUNE_VALIDATION_IMPLEMENTATION) && !defined(PORTABLE_TUNE_VALIDATION_IMPLEMENTED)
#define PORTABLE_TUNE_VALIDATION_IMPLEMENTED
#include <float.h>
static int pv_finite(float x) {return x==x&&x<=FLT_MAX&&x>=-FLT_MAX;}
static float pv_abs(float x) {return x<0.0f?-x:x;}
static unsigned long pv_elapsed(unsigned long a,unsigned long b)
{return (a-b)&0xFFFFFFFFUL;}
static int pv_fail(PortableTuneValidation *v, unsigned char reason)
{v->state=PORTABLE_VALIDATION_FAIL; v->reason=reason; return v->state;}
int portable_validation_start(PortableTuneValidation *v,
    const PortableTuneValidationConfig *c,unsigned long now_ms)
{
    if(!v) return 0;
    v->state=PORTABLE_VALIDATION_FAIL; v->reason=1;
    if(!c||!pv_finite(c->target)||!pv_finite(c->tolerance)
       ||!pv_finite(c->minimum_inside_fraction)||!pv_finite(c->maximum_peak_to_peak)
       ||!pv_finite(c->maximum_mean_error)||!pv_finite(c->maximum_overshoot)
       ||c->tolerance<=0.0f||c->minimum_inside_fraction<0.0f
       ||c->minimum_inside_fraction>1.0f||c->maximum_peak_to_peak<0.0f
       ||c->maximum_mean_error<0.0f||c->maximum_overshoot<0.0f
       ||c->window_ms==0UL||c->max_gap_ms==0UL
       ||c->warmup_ms>0x3FFFFFFFUL||c->window_ms>0x3FFFFFFFUL
       ||c->max_gap_ms>c->window_ms) return 0;
    v->cfg=*c; v->start_ms=now_ms&0xFFFFFFFFUL;
    v->sample_ms=v->start_ms; v->tick_ms=v->start_ms;
    v->count=0UL; v->inside=0UL; v->have_sample=0;
    v->minimum=0.0f; v->maximum=0.0f; v->mean=0.0f;
    v->peak=-FLT_MAX; v->last=0.0f;
    v->reason=0; v->state=PORTABLE_VALIDATION_RUNNING;
    return 1;
}
int portable_validation_tick(PortableTuneValidation *v,unsigned long now_ms,
    unsigned char permit)
{
    unsigned long now;
    if(!v) return PORTABLE_VALIDATION_IDLE;
    if(!permit && (v->state==PORTABLE_VALIDATION_RUNNING||v->state==PORTABLE_VALIDATION_PASS))
        return pv_fail(v,2);
    if(v->state!=PORTABLE_VALIDATION_RUNNING) return v->state;
    now=now_ms&0xFFFFFFFFUL;
    if(pv_elapsed(now,v->tick_ms)>0x7FFFFFFFUL) return pv_fail(v,3);
    v->tick_ms=now;
    if(pv_elapsed(now,v->sample_ms)>v->cfg.max_gap_ms) return pv_fail(v,4);
    return v->state;
}
int portable_validation_update(PortableTuneValidation *v,float y,
    unsigned long now_ms,unsigned char permit)
{
    unsigned long now, elapsed;
    float fraction;
    if(!v) return PORTABLE_VALIDATION_IDLE;
    portable_validation_tick(v,now_ms,permit);
    if(v->state!=PORTABLE_VALIDATION_RUNNING) return v->state;
    if(!pv_finite(y)) return pv_fail(v,5);
    now=now_ms&0xFFFFFFFFUL;
    if(v->have_sample&&now==v->sample_ms) return v->state;
    v->sample_ms=now; v->have_sample=1; v->last=y;
    if(y>v->peak) v->peak=y;
    if(y-v->cfg.target>v->cfg.maximum_overshoot) return pv_fail(v,6);
    elapsed=pv_elapsed(now,v->start_ms);
    if(elapsed<v->cfg.warmup_ms) return v->state;
    if(!v->count) {v->minimum=y; v->maximum=y;}
    if(y<v->minimum) v->minimum=y;
    if(y>v->maximum) v->maximum=y;
    if(v->count==ULONG_MAX) return pv_fail(v,5);
    v->count++;
    v->mean+=(y-v->mean)/(float)v->count;
    if(!pv_finite(v->mean)) return pv_fail(v,5);
    if(pv_abs(y-v->cfg.target)<=v->cfg.tolerance) v->inside++;
    if(elapsed>=v->cfg.warmup_ms+v->cfg.window_ms) {
        if(v->count<2UL) return pv_fail(v,7);
        fraction=(float)v->inside/(float)v->count;
        if(v->maximum-v->minimum>v->cfg.maximum_peak_to_peak
           ||pv_abs(v->mean-v->cfg.target)>v->cfg.maximum_mean_error
           ||fraction<v->cfg.minimum_inside_fraction) return pv_fail(v,7);
        v->state=PORTABLE_VALIDATION_PASS;
    }
    return v->state;
}
#endif

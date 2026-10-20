/* 通用继电反馈自整定，C89 单头模块。
 * 仅在一个 .c 定义 PORTABLE_TUNE_IMPLEMENTATION 后包含本文件。
 * 固定内存、非阻塞、无硬件/操作系统/Flash依赖。正增益对象，单向输出。
 * 完成表示取得候选参数，不代表实机闭环验收通过。
 */
#ifndef PORTABLE_TUNE_H
#define PORTABLE_TUNE_H
#include <limits.h>
#if ULONG_MAX < 4294967295UL
#error Portable_tune_requires_at_least_32_bit_unsigned_long
#endif

#define PORTABLE_TUNE_VERSION 100
#define PORTABLE_TUNE_MAX_CYCLES 6
#define PORTABLE_TUNE_IDLE 0
#define PORTABLE_TUNE_SETTLING 1
#define PORTABLE_TUNE_HIGH 2
#define PORTABLE_TUNE_LOW 3
#define PORTABLE_TUNE_CANDIDATE 4
#define PORTABLE_TUNE_FAILED 5
#define PORTABLE_TUNE_CANCELLED 6

#define PORTABLE_TUNE_OK 0
#define PORTABLE_TUNE_BAD_CONFIG 1
#define PORTABLE_TUNE_BAD_SAMPLE 2
#define PORTABLE_TUNE_INTERLOCK 3
#define PORTABLE_TUNE_NO_SAMPLE 4
#define PORTABLE_TUNE_CLOCK 5
#define PORTABLE_TUNE_TOTAL_TIMEOUT 6
#define PORTABLE_TUNE_SETTLE_TIMEOUT 7
#define PORTABLE_TUNE_HALF_TIMEOUT 8
#define PORTABLE_TUNE_OUTPUT_MISMATCH 9
#define PORTABLE_TUNE_OUT_OF_RANGE 10
#define PORTABLE_TUNE_POOR_OSCILLATION 11
#define PORTABLE_TUNE_GAIN_RANGE 12
#define PORTABLE_TUNE_USER_CANCEL 13

typedef struct {
    float target;              /* 整定目标，输入物理单位 */
    float bias, relay_step;    /* 维持功率与对称扰动半幅，必须人工/上层确定 */
    float output_min, output_max; /* 当前要求 output_min=0 */
    float hysteresis;          /* 温度切换半带宽，必须>0且大于有效噪声 */
    float input_min, input_max; /* 实验允许测量范围，越界立即停止 */
    float settle_band;         /* 等待阶段测量相对目标允许偏差 */
    float settle_slope;        /* 等待阶段允许变化率绝对值，输入单位/秒 */
    float settle_span;         /* 整个稳定等待窗口允许峰峰值，防慢漂移 */
    float min_amplitude, max_amplitude; /* 可接受温度半振幅 */
    float amplitude_tolerance, period_tolerance; /* 周期间相对极差 */
    float center_tolerance;    /* 各周期温度中心相对目标允许偏差 */
    float center_spread_limit; /* 最近周期中心允许极差，防逐周期漂移 */
    float max_hysteresis_ratio; /* hysteresis/半振幅上限，<=0.5 */
    float output_tolerance;    /* 实际已施加输出与上次命令允许误差 */
    float kp_factor, ti_factor, td_factor; /* Kp=c*Ku, Ti=c*Pu, Td=c*Pu */
    float kp_min, kp_max, ki_min, ki_max, kd_max; /* 候选增益允许范围 */
    unsigned long settle_ms, settle_timeout_ms;
    unsigned long min_half_ms, half_timeout_ms;
    unsigned long min_period_ms, max_period_ms;
    unsigned long max_sample_gap_ms, total_timeout_ms;
    unsigned long actuator_grace_ms; /* 命令切换后允许的有限执行延迟 */
    unsigned char discard_cycles;   /* 丢弃启动过渡完整周期数 */
    unsigned char required_cycles;  /* 连续质量检查窗口，3~6 */
    unsigned char max_cycles;       /* 最大总周期数，未收敛则失败 */
} PortableTuneConfig;

typedef struct {
    float kp, ki, kd, ku, period_s, amplitude, output_half_span;
    float amplitude_spread, period_spread, center_error, center_spread;
    unsigned char cycles;
    unsigned char valid; /* 仅CANDIDATE时为1；故障/取消会撤销 */
} PortableTuneResult;

typedef struct {
    PortableTuneConfig cfg;
    PortableTuneResult result;
    float output, last_measurement, window_min, window_max;
    float settle_min, settle_max;
    float high_sum, low_sum;
    float amplitudes[PORTABLE_TUNE_MAX_CYCLES];
    float periods[PORTABLE_TUNE_MAX_CYCLES];
    float centers[PORTABLE_TUNE_MAX_CYCLES];
    float spans[PORTABLE_TUNE_MAX_CYCLES];
    unsigned long start_ms, sample_ms, tick_ms, switch_ms;
    unsigned long stable_ms, cycle_ms;
    unsigned long high_count, low_count;
    unsigned char state, reason, initialized;
    unsigned char have_sample, stable, have_cycle, cycle_high_seen;
    unsigned char awaiting_output;
    unsigned char total_cycles, stored, write_index;
} PortableTune;

/* 零输出是本模块唯一退出输出；失败/取消/取得候选后必须由上层写入执行器。 */
void portable_tune_defaults(PortableTuneConfig *c);
int portable_tune_config_valid(const PortableTuneConfig *c);
void portable_tune_reset(PortableTune *t);
int portable_tune_active(const PortableTune *t);
int portable_tune_start(PortableTune *t, const PortableTuneConfig *c,
                        unsigned long now_ms);
/* tick必须由主循环周期调用，即便没有新传感器样本也要调用。
 * now_ms为单调32位毫秒计数，允许约49.7天自然回绕。
 * permit=0立即撤销候选并退出；模块不直接写硬件。
 */
int portable_tune_tick(PortableTune *t, unsigned long now_ms,
                       unsigned char permit);
/* 仅新鲜有效样本调用。applied_output是本拍之前真正已施加的输出，
 * 不是刚发出的新命令。返回当前状态；命令从t->output读取并统一下发。
 */
int portable_tune_update(PortableTune *t, float measurement,
                         float applied_output, unsigned long now_ms,
                         unsigned char permit);
void portable_tune_cancel(PortableTune *t);
int portable_tune_get_result(const PortableTune *t, PortableTuneResult *r);
const char *portable_tune_reason(unsigned char reason);
#endif

#if defined(PORTABLE_TUNE_IMPLEMENTATION) && !defined(PORTABLE_TUNE_IMPLEMENTED)
#define PORTABLE_TUNE_IMPLEMENTED
#include <float.h>

#define PT_MASK 0xFFFFFFFFUL
#define PT_HALF 0x7FFFFFFFUL
#define PT_PI 3.14159265358979323846f
static float pt_abs(float x) { return x<0.0f ? -x : x; }
static int pt_finite(float x) { return x==x && x<=FLT_MAX && x>=-FLT_MAX; }
static unsigned long pt_elapsed(unsigned long a, unsigned long b)
{ return (a-b)&PT_MASK; }
static int pt_duration(unsigned long n) { return n>0UL && n<=PT_HALF; }
static int pt_fail(PortableTune *t, unsigned char reason)
{
    t->state=PORTABLE_TUNE_FAILED; t->reason=reason;
    t->output=0.0f; t->result.valid=0;
    return t->state;
}

void portable_tune_defaults(PortableTuneConfig *c)
{
    if(!c) return;
    /* 这些是示例量程/门槛，不是适配任意设备的安全参数。 */
    c->target=100.0f; c->bias=79.0f; c->relay_step=8.0f;
    c->output_min=0.0f; c->output_max=100.0f;
    c->hysteresis=0.3f; c->input_min=0.0f; c->input_max=120.0f;
    c->settle_band=2.0f; c->settle_slope=0.03f; c->settle_span=0.5f;
    c->min_amplitude=0.7f; c->max_amplitude=5.0f;
    c->amplitude_tolerance=0.15f; c->period_tolerance=0.15f;
    c->center_tolerance=2.0f; c->center_spread_limit=0.5f;
    c->max_hysteresis_ratio=0.45f;
    c->output_tolerance=0.6f;
    /* 工程保守起点，不冒称标准Z-N或保证无超调；默认PI。 */
    c->kp_factor=0.2f; c->ti_factor=1.0f; c->td_factor=0.0f;
    c->kp_min=0.00001f; c->kp_max=1000.0f;
    c->ki_min=0.000101f; c->ki_max=100.0f; c->kd_max=10000.0f;
    c->settle_ms=60000UL; c->settle_timeout_ms=1200000UL;
    c->min_half_ms=2000UL; c->half_timeout_ms=600000UL;
    c->min_period_ms=10000UL; c->max_period_ms=900000UL;
    c->max_sample_gap_ms=2000UL; c->total_timeout_ms=7200000UL;
    c->actuator_grace_ms=1000UL;
    c->discard_cycles=2; c->required_cycles=4; c->max_cycles=16;
}

int portable_tune_config_valid(const PortableTuneConfig *c)
{
    if(!c) return 0;
    /* 所有浮点参数显式检查，不依赖结构布局或指针跨成员遍历。 */
    if(!pt_finite(c->target)||!pt_finite(c->bias)||!pt_finite(c->relay_step)
       ||!pt_finite(c->output_min)||!pt_finite(c->output_max)
       ||!pt_finite(c->hysteresis)||!pt_finite(c->input_min)
       ||!pt_finite(c->input_max)||!pt_finite(c->settle_band)
       ||!pt_finite(c->settle_slope)||!pt_finite(c->min_amplitude)
       ||!pt_finite(c->settle_span)||!pt_finite(c->center_spread_limit)
       ||!pt_finite(c->max_amplitude)||!pt_finite(c->amplitude_tolerance)
       ||!pt_finite(c->period_tolerance)||!pt_finite(c->center_tolerance)
       ||!pt_finite(c->max_hysteresis_ratio)||!pt_finite(c->output_tolerance)
       ||!pt_finite(c->kp_factor)||!pt_finite(c->ti_factor)
       ||!pt_finite(c->td_factor)||!pt_finite(c->kp_min)
       ||!pt_finite(c->kp_max)||!pt_finite(c->ki_min)
       ||!pt_finite(c->ki_max)||!pt_finite(c->kd_max)) return 0;
    if(c->output_min!=0.0f || c->output_max<=0.0f
       || c->output_tolerance<0.0f || c->relay_step<=0.0f
       || c->relay_step<=2.0f*c->output_tolerance
       || c->bias-c->relay_step<c->output_min
       || !pt_finite(c->bias+c->relay_step)
       || c->bias+c->relay_step>c->output_max) return 0;
    if(c->hysteresis<=0.0f || c->input_min>=c->input_max
       || c->settle_band<c->hysteresis || c->settle_slope<=0.0f || c->settle_span<=0.0f
       || c->min_amplitude<=c->hysteresis
       || c->max_amplitude<c->min_amplitude
       || !pt_finite(c->target+c->max_amplitude)
       || !pt_finite(c->target-c->max_amplitude)
       || c->target-c->max_amplitude<=c->input_min
       || c->target+c->max_amplitude>=c->input_max) return 0;
    if(c->amplitude_tolerance<=0.0f || c->amplitude_tolerance>1.0f
       || c->period_tolerance<=0.0f || c->period_tolerance>1.0f
       || c->center_tolerance<0.0f || c->center_spread_limit<0.0f
       || c->max_hysteresis_ratio<=0.0f
       || c->max_hysteresis_ratio>0.5f || c->kp_factor<=0.0f
       || c->ti_factor<=0.0f || c->td_factor<0.0f
       || c->kp_min<=0.0f || c->kp_max<c->kp_min
       || c->ki_min<=0.0f || c->ki_max<c->ki_min || c->kd_max<0.0f) return 0;
    if(c->hysteresis/c->max_hysteresis_ratio>c->max_amplitude) return 0;
    if(!pt_duration(c->settle_ms)||!pt_duration(c->settle_timeout_ms)
       ||!pt_duration(c->min_half_ms)||!pt_duration(c->half_timeout_ms)
       ||!pt_duration(c->min_period_ms)||!pt_duration(c->max_period_ms)
       ||!pt_duration(c->max_sample_gap_ms)||!pt_duration(c->total_timeout_ms)
       || c->actuator_grace_ms>PT_HALF) return 0;
    if(c->settle_timeout_ms<c->settle_ms
       || c->half_timeout_ms<c->min_half_ms
       || c->max_period_ms<c->min_period_ms
       || c->min_period_ms<2UL*c->min_half_ms
       || c->actuator_grace_ms>=c->min_half_ms
       || c->total_timeout_ms<=c->settle_timeout_ms
       || c->required_cycles<3 || c->required_cycles>PORTABLE_TUNE_MAX_CYCLES
       || c->discard_cycles>20 || c->max_cycles>100
       || c->max_cycles<c->discard_cycles+c->required_cycles) return 0;
    return 1;
}

void portable_tune_reset(PortableTune *t)
{
    unsigned char i;
    if(!t) return;
    t->state=PORTABLE_TUNE_IDLE; t->reason=PORTABLE_TUNE_OK;
    t->initialized=1; t->output=0.0f; t->result.valid=0;
    t->result.kp=0.0f; t->result.ki=0.0f; t->result.kd=0.0f;
    t->result.ku=0.0f; t->result.period_s=0.0f; t->result.amplitude=0.0f;
    t->result.output_half_span=0.0f; t->result.amplitude_spread=0.0f;
    t->result.period_spread=0.0f; t->result.center_error=0.0f;
    t->result.center_spread=0.0f; t->settle_min=0.0f; t->settle_max=0.0f;
    t->result.cycles=0; t->last_measurement=0.0f;
    t->window_min=0.0f; t->window_max=0.0f;
    t->high_sum=0.0f; t->low_sum=0.0f;
    t->high_count=0UL; t->low_count=0UL;
    t->start_ms=0UL; t->sample_ms=0UL; t->tick_ms=0UL;
    t->switch_ms=0UL; t->stable_ms=0UL; t->cycle_ms=0UL;
    t->have_sample=0; t->stable=0; t->have_cycle=0; t->cycle_high_seen=0;
    t->awaiting_output=0;
    t->total_cycles=0; t->stored=0; t->write_index=0;
    for(i=0;i<PORTABLE_TUNE_MAX_CYCLES;i++) {
        t->amplitudes[i]=0.0f; t->periods[i]=0.0f;
        t->centers[i]=0.0f; t->spans[i]=0.0f;
    }
}

int portable_tune_active(const PortableTune *t)
{
    return t && t->initialized && (t->state==PORTABLE_TUNE_SETTLING
        || t->state==PORTABLE_TUNE_HIGH || t->state==PORTABLE_TUNE_LOW);
}

int portable_tune_start(PortableTune *t, const PortableTuneConfig *c,
                        unsigned long now_ms)
{
    /* reset不改cfg，允许使用已保存配置重新开始，但会清旧结果。 */
    if(!t) return 0;
    if(!portable_tune_config_valid(c)) {
        portable_tune_reset(t); pt_fail(t,PORTABLE_TUNE_BAD_CONFIG); return 0;
    }
    t->cfg=*c;
    portable_tune_reset(t);
    t->start_ms=now_ms&PT_MASK; t->sample_ms=t->start_ms;
    t->tick_ms=t->start_ms; t->switch_ms=t->start_ms;
    t->state=PORTABLE_TUNE_SETTLING; t->output=t->cfg.bias;
    t->awaiting_output=1;
    return 1;
}

int portable_tune_tick(PortableTune *t, unsigned long now_ms,
                       unsigned char permit)
{
    unsigned long now, d;
    if(!t || !t->initialized) return PORTABLE_TUNE_IDLE;
    if(!permit && (portable_tune_active(t)||t->state==PORTABLE_TUNE_CANDIDATE))
        return pt_fail(t,PORTABLE_TUNE_INTERLOCK);
    if(!portable_tune_active(t)) return t->state;
    now=now_ms&PT_MASK;
    if(pt_elapsed(now,t->tick_ms)>PT_HALF) return pt_fail(t,PORTABLE_TUNE_CLOCK);
    t->tick_ms=now;
    if(t->awaiting_output && pt_elapsed(now,t->switch_ms)>t->cfg.actuator_grace_ms)
        return pt_fail(t,PORTABLE_TUNE_OUTPUT_MISMATCH);
    d=pt_elapsed(now,t->sample_ms);
    if(d>t->cfg.max_sample_gap_ms) return pt_fail(t,PORTABLE_TUNE_NO_SAMPLE);
    if(pt_elapsed(now,t->start_ms)>=t->cfg.total_timeout_ms)
        return pt_fail(t,PORTABLE_TUNE_TOTAL_TIMEOUT);
    if(t->state==PORTABLE_TUNE_SETTLING) {
        if(pt_elapsed(now,t->start_ms)>=t->cfg.settle_timeout_ms)
            return pt_fail(t,PORTABLE_TUNE_SETTLE_TIMEOUT);
    } else if(pt_elapsed(now,t->switch_ms)>=t->cfg.half_timeout_ms)
        return pt_fail(t,PORTABLE_TUNE_HALF_TIMEOUT);
    return t->state;
}

static int pt_candidate(PortableTune *t)
{
    unsigned char i;
    float amin, amax, pmin, pmax, amean, pmean, hmean, center, cmin, cmax;
    float kp, ki, kd, ku, ti;
    if(t->stored<t->cfg.required_cycles) return 0;
    amin=t->amplitudes[0]; amax=amin; pmin=t->periods[0]; pmax=pmin;
    cmin=t->centers[0]; cmax=cmin;
    amean=0.0f; pmean=0.0f; hmean=0.0f; center=0.0f;
    for(i=0;i<t->cfg.required_cycles;i++) {
        if(t->amplitudes[i]<amin) amin=t->amplitudes[i];
        if(t->amplitudes[i]>amax) amax=t->amplitudes[i];
        if(t->periods[i]<pmin) pmin=t->periods[i];
        if(t->periods[i]>pmax) pmax=t->periods[i];
        if(t->centers[i]<cmin) cmin=t->centers[i];
        if(t->centers[i]>cmax) cmax=t->centers[i];
        amean+=t->amplitudes[i]; pmean+=t->periods[i]; hmean+=t->spans[i];
        if(pt_abs(t->centers[i]-t->cfg.target)>center)
            center=pt_abs(t->centers[i]-t->cfg.target);
    }
    amean/=t->cfg.required_cycles; pmean/=t->cfg.required_cycles;
    hmean/=t->cfg.required_cycles;
    if(!pt_finite(amean)||!pt_finite(pmean)||!pt_finite(hmean)
       || amean<=0.0f || pmean<=0.0f || hmean<=0.0f)
        return pt_fail(t,PORTABLE_TUNE_POOR_OSCILLATION);
    t->result.amplitude_spread=(amax-amin)/amean;
    t->result.period_spread=(pmax-pmin)/pmean;
    t->result.center_error=center;
    t->result.center_spread=cmax-cmin;
    if(amin<t->cfg.min_amplitude || amax>t->cfg.max_amplitude
       || pmin*1000.0f<(float)t->cfg.min_period_ms
       || pmax*1000.0f>(float)t->cfg.max_period_ms
       || t->cfg.hysteresis/amin>t->cfg.max_hysteresis_ratio
       || t->result.amplitude_spread>t->cfg.amplitude_tolerance
       || t->result.period_spread>t->cfg.period_tolerance
       || center>t->cfg.center_tolerance
       || t->result.center_spread>t->cfg.center_spread_limit) return 0;
    /* 使用经过执行器回读核对的高低输出均值计算半幅，非虚构h。 */
    ku=4.0f*hmean/(PT_PI*amean);
    kp=t->cfg.kp_factor*ku; ti=t->cfg.ti_factor*pmean;
    if(!pt_finite(ku)||!pt_finite(kp)||!pt_finite(ti)||ti<=0.0f)
        return pt_fail(t,PORTABLE_TUNE_GAIN_RANGE);
    ki=kp/ti; kd=kp*(t->cfg.td_factor*pmean);
    if(!pt_finite(ki)||!pt_finite(kd)||kp<t->cfg.kp_min||kp>t->cfg.kp_max
       ||ki<t->cfg.ki_min||ki>t->cfg.ki_max||kd>t->cfg.kd_max)
        return pt_fail(t,PORTABLE_TUNE_GAIN_RANGE);
    t->result.kp=kp; t->result.ki=ki; t->result.kd=kd; t->result.ku=ku;
    t->result.period_s=pmean; t->result.amplitude=amean;
    t->result.output_half_span=hmean; t->result.cycles=t->stored;
    t->result.valid=1; t->output=0.0f;
    t->state=PORTABLE_TUNE_CANDIDATE; t->reason=PORTABLE_TUNE_OK;
    return t->state;
}

int portable_tune_update(PortableTune *t, float y, float applied,
                         unsigned long now_ms, unsigned char permit)
{
    unsigned long now, delta, phase;
    float slope, amp, center, span;
    if(!t || !t->initialized) return PORTABLE_TUNE_IDLE;
    /* 若本拍在宽限期内确认执行器已到位，先确认再运行独立看门狗。
     * 时间倒退/超时仍由tick裁定，不能用迟到回读复活过期命令。
     */
    now=now_ms&PT_MASK;
    if(portable_tune_active(t)&&pt_finite(applied)
       &&pt_abs(applied-t->output)<=t->cfg.output_tolerance
       &&pt_elapsed(now,t->switch_ms)<=t->cfg.actuator_grace_ms)
        t->awaiting_output=0;
    portable_tune_tick(t,now_ms,permit);
    if(!portable_tune_active(t)) return t->state;
    now=now_ms&PT_MASK;
    if(!pt_finite(y)||!pt_finite(applied)) return pt_fail(t,PORTABLE_TUNE_BAD_SAMPLE);
    if(y<=t->cfg.input_min||y>=t->cfg.input_max)
        return pt_fail(t,PORTABLE_TUNE_OUT_OF_RANGE);
    if(applied<t->cfg.output_min-t->cfg.output_tolerance
       ||applied>t->cfg.output_max+t->cfg.output_tolerance)
        return pt_fail(t,PORTABLE_TUNE_OUTPUT_MISMATCH);
    /* 相同毫秒的重复样本不推进波形；外层仍要按帧序号做去重。 */
    if(t->have_sample && now==t->sample_ms) return t->state;
    delta=pt_elapsed(now,t->sample_ms);
    if(pt_abs(applied-t->output)>t->cfg.output_tolerance) {
        if(pt_elapsed(now,t->switch_ms)>t->cfg.actuator_grace_ms)
            return pt_fail(t,PORTABLE_TUNE_OUTPUT_MISMATCH);
        /* 允许短暂执行延迟，但不把旧输出计入继电高/低幅值。 */
        t->sample_ms=now; t->have_sample=1; t->stable=0;
        t->awaiting_output=1;
        t->last_measurement=y;
        return t->state;
    }
    t->awaiting_output=0;
    slope=0.0f;
    if(t->have_sample && delta>0UL)
        slope=pt_abs(y-t->last_measurement)*1000.0f/(float)delta;
    t->sample_ms=now; t->last_measurement=y;
    if(t->state==PORTABLE_TUNE_SETTLING) {
        if(t->have_sample && pt_abs(y-t->cfg.target)<=t->cfg.settle_band
           && slope<=t->cfg.settle_slope) {
            if(!t->stable) {
                t->stable=1; t->stable_ms=now;
                t->settle_min=y; t->settle_max=y;
            }
            if(y<t->settle_min) t->settle_min=y;
            if(y>t->settle_max) t->settle_max=y;
            if(t->settle_max-t->settle_min>t->cfg.settle_span) {
                t->stable_ms=now; t->settle_min=y; t->settle_max=y;
            }
            if(pt_elapsed(now,t->stable_ms)>=t->cfg.settle_ms) {
                t->state=PORTABLE_TUNE_HIGH; t->switch_ms=now;
                t->output=t->cfg.bias+t->cfg.relay_step;
                t->awaiting_output=1;
                t->window_min=y; t->window_max=y;
            }
        } else t->stable=0;
        t->have_sample=1;
        return t->state;
    }
    t->have_sample=1;
    if(y<t->window_min) t->window_min=y;
    if(y>t->window_max) t->window_max=y;
    if(t->window_max-t->window_min>2.0f*t->cfg.max_amplitude)
        return pt_fail(t,PORTABLE_TUNE_POOR_OSCILLATION);
    if(t->state==PORTABLE_TUNE_HIGH) {
        t->high_sum+=applied; t->high_count++;
    } else {t->low_sum+=applied; t->low_count++;}
    if(!pt_finite(t->high_sum)||!pt_finite(t->low_sum)
       ||t->high_count==ULONG_MAX||t->low_count==ULONG_MAX)
        return pt_fail(t,PORTABLE_TUNE_BAD_SAMPLE);
    phase=pt_elapsed(now,t->switch_ms);
    if(phase<t->cfg.min_half_ms) return t->state;
    if(t->state==PORTABLE_TUNE_HIGH && y>=t->cfg.target+t->cfg.hysteresis) {
        t->state=PORTABLE_TUNE_LOW; t->switch_ms=now;
        t->output=t->cfg.bias-t->cfg.relay_step;
        t->awaiting_output=1;
        t->cycle_high_seen=1;
    } else if(t->state==PORTABLE_TUNE_LOW && y<=t->cfg.target-t->cfg.hysteresis) {
        if(t->have_cycle && t->cycle_high_seen && t->high_count && t->low_count) {
            t->total_cycles++;
            amp=0.5f*(t->window_max-t->window_min);
            center=t->window_min+amp;
            span=0.5f*(t->high_sum/(float)t->high_count-t->low_sum/(float)t->low_count);
            if(t->total_cycles>t->cfg.discard_cycles) {
                t->amplitudes[t->write_index]=amp;
                t->centers[t->write_index]=center;
                t->periods[t->write_index]=(float)pt_elapsed(now,t->cycle_ms)/1000.0f;
                t->spans[t->write_index]=span;
                t->write_index++;
                if(t->write_index>=t->cfg.required_cycles) t->write_index=0;
                if(t->stored<t->cfg.required_cycles) t->stored++;
                pt_candidate(t);
                if(!portable_tune_active(t)) return t->state;
            }
            if(t->total_cycles>=t->cfg.max_cycles)
                return pt_fail(t,PORTABLE_TUNE_POOR_OSCILLATION);
        }
        t->state=PORTABLE_TUNE_HIGH; t->switch_ms=now;
        t->output=t->cfg.bias+t->cfg.relay_step;
        t->awaiting_output=1;
        t->have_cycle=1; t->cycle_high_seen=0; t->cycle_ms=now;
        t->window_min=y; t->window_max=y;
        t->high_sum=0.0f; t->low_sum=0.0f;
        t->high_count=0UL; t->low_count=0UL;
    }
    return t->state;
}

void portable_tune_cancel(PortableTune *t)
{
    if(!t || !t->initialized) return;
    t->output=0.0f; t->result.valid=0;
    t->state=PORTABLE_TUNE_CANCELLED; t->reason=PORTABLE_TUNE_USER_CANCEL;
}

int portable_tune_get_result(const PortableTune *t, PortableTuneResult *r)
{
    if(!t||!r||!t->initialized||t->state!=PORTABLE_TUNE_CANDIDATE||!t->result.valid)
        return 0;
    *r=t->result; return 1;
}

const char *portable_tune_reason(unsigned char reason)
{
    /* ASCII错误字串避免依赖设备字库，中文对照见说明。 */
    switch(reason) {
    case PORTABLE_TUNE_OK: return "ok";
    case PORTABLE_TUNE_BAD_CONFIG: return "invalid configuration";
    case PORTABLE_TUNE_BAD_SAMPLE: return "invalid sample";
    case PORTABLE_TUNE_INTERLOCK: return "interlock denied";
    case PORTABLE_TUNE_NO_SAMPLE: return "sensor timeout";
    case PORTABLE_TUNE_CLOCK: return "clock reversed";
    case PORTABLE_TUNE_TOTAL_TIMEOUT: return "total timeout";
    case PORTABLE_TUNE_SETTLE_TIMEOUT: return "settling timeout";
    case PORTABLE_TUNE_HALF_TIMEOUT: return "half cycle timeout";
    case PORTABLE_TUNE_OUTPUT_MISMATCH: return "output not applied";
    case PORTABLE_TUNE_OUT_OF_RANGE: return "measurement out of range";
    case PORTABLE_TUNE_POOR_OSCILLATION: return "oscillation not usable";
    case PORTABLE_TUNE_GAIN_RANGE: return "candidate gains out of range";
    case PORTABLE_TUNE_USER_CANCEL: return "cancelled";
    default: return "unknown";
    }
}
#endif

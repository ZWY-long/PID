/* 工程接入示例，不可不经改配置就连接现场执行器。
 * 此.c独立持有全部实现；若移入main.c，把下列实现宏也移入同一编译单元。
 * 所有返回功率均必须由项目统一写到真实硬件；停热不走PID限速。
 */
#define PORTABLE_PID_IMPLEMENTATION
#include "../通用比例积分微分.h"
#define PORTABLE_TUNE_IMPLEMENTATION
#include "../通用自整定.h"
#define PORTABLE_TUNE_VALIDATION_IMPLEMENTATION
#include "../通用自整定验收.h"

#if defined(__C51__)
#define EXAMPLE_RAM xdata
#else
#define EXAMPLE_RAM
#endif

#define EXAMPLE_OFF 0
#define EXAMPLE_TUNING 1
#define EXAMPLE_WAITING 2
#define EXAMPLE_TRIAL 3
#define EXAMPLE_ACCEPTED 4

static PortableTune EXAMPLE_RAM example_tune;
static PortableTuneConfig EXAMPLE_RAM example_tune_cfg;
static PortableTuneValidation EXAMPLE_RAM example_validation;
static PortableTuneValidationConfig EXAMPLE_RAM example_validation_cfg;
static PortablePid EXAMPLE_RAM example_pid;
static PortablePidConfig EXAMPLE_RAM example_pid_cfg;
static PortablePidConfig EXAMPLE_RAM example_saved_cfg;
static PortableTuneResult EXAMPLE_RAM example_candidate;
static unsigned char example_mode=EXAMPLE_OFF;
static unsigned long example_previous_ms=0UL;
static float example_target=100.0f;
void example_cancel(void);

void example_init(void)
{
    portable_tune_reset(&example_tune);
    portable_pid_reset(&example_pid);
    example_pid_cfg=portable_pid_heater_config();
    example_saved_cfg=example_pid_cfg;
    example_validation.state=PORTABLE_VALIDATION_IDLE;
    example_mode=EXAMPLE_OFF;
}

/* 外部先用已有控制器/人工控制接近工作点，测出近稳态维持功率。
 * bias必须随新设备/目标/负载重新确定；不能固定使用79%。
 */
int example_start_tune(float target,float bias,float measured,
    unsigned long now_ms,unsigned char permit)
{
    if(!permit) {example_cancel(); return 0;}
    if(example_mode==EXAMPLE_TUNING||example_mode==EXAMPLE_TRIAL) return 0;
    example_saved_cfg=example_pid_cfg;
    portable_tune_defaults(&example_tune_cfg);
    example_tune_cfg.target=target; example_tune_cfg.bias=bias;
    if(!portable_pid_finite(measured)||measured<=example_tune_cfg.input_min
       ||measured>=example_tune_cfg.input_max) return 0;
    /* 在此针对项目配置继电幅度、温度边界、噪声门槛和全部时间限值。
     * 默认PI：td_factor=0。试验PID时可设0.125，结果仍须重新验证。
     */
    if(!portable_tune_start(&example_tune,&example_tune_cfg,now_ms)) {
        example_mode=EXAMPLE_OFF; return 0;
    }
    example_target=target; example_mode=EXAMPLE_TUNING;
    return 1; /* 立即把example_tune.output下发，随后持续调用tick和新样本入口。 */
}

/* 由操作界面明确请求试跑候选；不会在取得候选后自动修改在用参数。 */
int example_begin_trial(float measured,float actual_output,unsigned long now_ms,
    unsigned char permit)
{
    float err, i_term;
    if(!permit) {example_cancel(); return 0;}
    if(example_mode!=EXAMPLE_WAITING
       ||!portable_tune_get_result(&example_tune,&example_candidate)
       ||!portable_pid_finite(measured)||!portable_pid_finite(actual_output)
       ||measured<=example_tune_cfg.input_min||measured>=example_tune_cfg.input_max
       ||actual_output<0.0f||actual_output>example_pid_cfg.output_max) return 0;
    example_pid_cfg.kp=example_candidate.kp;
    example_pid_cfg.ki=example_candidate.ki;
    example_pid_cfg.kd=example_candidate.kd;
    if(!portable_pid_config_valid(&example_pid_cfg)) {
        example_pid_cfg=example_saved_cfg; return 0;
    }
    portable_pid_reset(&example_pid);
    err=example_target-measured;
    if(err<example_pid_cfg.p_deadband&&err>-example_pid_cfg.p_deadband) err=0.0f;
    i_term=actual_output-example_pid_cfg.kp*err;
    if(i_term<0.0f) i_term=0.0f;
    if(i_term>example_pid_cfg.output_max) i_term=example_pid_cfg.output_max;
    example_pid.integral=i_term/example_pid_cfg.ki;
    example_pid.output=actual_output; example_pid.last_out=actual_output;
    example_pid.last_temp=measured; example_pid.have_temp=1;
    example_previous_ms=now_ms&0xFFFFFFFFUL;
    example_validation_cfg.target=example_target;
    example_validation_cfg.tolerance=0.5f;
    example_validation_cfg.minimum_inside_fraction=1.0f;
    example_validation_cfg.maximum_peak_to_peak=1.0f;
    example_validation_cfg.maximum_mean_error=0.5f;
    example_validation_cfg.maximum_overshoot=2.0f;
    example_validation_cfg.warmup_ms=1500000UL; /* 示例：热透25分钟 */
    example_validation_cfg.window_ms=300000UL; /* 再验收5分钟 */
    example_validation_cfg.max_gap_ms=(unsigned long)(example_pid_cfg.max_dt*1000.0f);
    if(!portable_validation_start(&example_validation,&example_validation_cfg,now_ms)) {
        example_pid_cfg=example_saved_cfg; example_mode=EXAMPLE_OFF; return 0;
    }
    example_mode=EXAMPLE_TRIAL;
    return 1;
}

void example_cancel(void)
{
    portable_tune_cancel(&example_tune);
    example_pid_cfg=example_saved_cfg;
    portable_pid_reset(&example_pid); example_mode=EXAMPLE_OFF;
    /* 项目必须同时立即把执行器写0。这里只复位算法内存。 */
}

/* 主循环周期维护：即使没有新温度也要调用，返回值统一下发。
 * permit应包含：探头有效、超温允许、通信正常、现场安全联锁等。
 * 也必须在项目里设置持续工作的最终硬件/通信看门狗。
 */
float example_tick(unsigned long now_ms,unsigned char permit)
{
    if(!permit) {example_cancel(); return 0.0f;}
    if(example_mode==EXAMPLE_TUNING) {
        portable_tune_tick(&example_tune,now_ms,permit);
        if(!portable_tune_active(&example_tune)) example_mode=EXAMPLE_OFF;
        return example_tune.output;
    }
    if(example_mode==EXAMPLE_TRIAL||example_mode==EXAMPLE_ACCEPTED) {
        if(((now_ms-example_previous_ms)&0xFFFFFFFFUL)
           >(unsigned long)(example_pid_cfg.max_dt*1000.0f)) {
            example_cancel(); return 0.0f;
        }
        if(example_mode==EXAMPLE_TRIAL) {
            portable_validation_tick(&example_validation,now_ms,permit);
            if(example_validation.state==PORTABLE_VALIDATION_FAIL) {
                example_cancel(); return 0.0f;
            }
        }
        return example_pid.output;
    }
    return 0.0f;
}

/* 只在确认到达新的有效帧时调用；applied是此前已经施加的功率回读。 */
float example_on_new_sample(float measured,float applied,unsigned long now_ms,
    unsigned char permit)
{
    unsigned long elapsed;
    if(!permit) {example_cancel(); return 0.0f;}
    if(example_mode==EXAMPLE_TUNING) {
        portable_tune_update(&example_tune,measured,applied,now_ms,permit);
        if(example_tune.state==PORTABLE_TUNE_CANDIDATE) example_mode=EXAMPLE_WAITING;
        else if(!portable_tune_active(&example_tune)) example_mode=EXAMPLE_OFF;
        return example_tune.output;
    }
    if(example_mode!=EXAMPLE_TRIAL&&example_mode!=EXAMPLE_ACCEPTED) return 0.0f;
    elapsed=(now_ms-example_previous_ms)&0xFFFFFFFFUL;
    if(elapsed==0UL) return example_pid.output;
    if(elapsed>(unsigned long)(example_pid_cfg.max_dt*1000.0f)) {
        example_cancel(); return 0.0f;
    }
    if(elapsed<(unsigned long)(example_pid_cfg.min_dt*1000.0f))
        return example_pid.output;
    example_previous_ms=now_ms&0xFFFFFFFFUL;
    if(example_mode==EXAMPLE_TRIAL) {
        portable_validation_update(&example_validation,measured,now_ms,permit);
        if(example_validation.state==PORTABLE_VALIDATION_FAIL) {
            example_cancel(); return 0.0f;
        }
        if(example_validation.state==PORTABLE_VALIDATION_PASS)
            example_mode=EXAMPLE_ACCEPTED; /* 仅代表当前验收配置下的试跑通过 */
    }
    if(!portable_pid_step(&example_pid,&example_pid_cfg,example_target,measured,
                         (float)elapsed/1000.0f)) {example_cancel(); return 0.0f;}
    return example_pid.output;
}
/* Flash持久化由项目在EXAMPLE_ACCEPTED后独立实现；本示例不写Flash。 */

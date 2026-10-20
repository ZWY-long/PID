#include <stdio.h>
/* 用-include包含交付的接入示例，在同一编译单元执行真实实现。 */
static int good=0,bad=0;
#define CHECK(x) do {if(x)good++;else{bad++;printf("FAIL line %d\n",__LINE__);}}while(0)
static void candidate_fixture(void)
{
    example_init(); portable_tune_defaults(&example_tune_cfg);
    example_tune.state=PORTABLE_TUNE_CANDIDATE;
    example_tune.result.valid=1;
    example_tune.result.kp=2.0f;example_tune.result.ki=0.1f;
    example_tune.result.kd=0.0f;example_mode=EXAMPLE_WAITING;
}
int main(void)
{
    unsigned long t;
    float out;
    example_init();
    CHECK(example_start_tune(100.0f,79.0f,100.0f,0UL,1));
    out=example_on_new_sample(100.0f,79.0f,0UL,1);
    CHECK(out==79.0f&&example_mode==EXAMPLE_TUNING);
    CHECK(example_tick(100UL,0)==0.0f&&example_mode==EXAMPLE_OFF);
    CHECK(!example_start_tune(100.0f,79.0f,130.0f,0UL,1));
    candidate_fixture();CHECK(!example_begin_trial(100.0f,0.0f,0UL,0)&&example_mode==EXAMPLE_OFF);
    candidate_fixture();CHECK(!example_begin_trial(130.0f,0.0f,0UL,1));
    candidate_fixture();CHECK(example_begin_trial(100.0f,79.0f,0UL,1));
    CHECK(example_pid_cfg.kp==2.0f&&example_saved_cfg.kp==10.0f);
    CHECK(example_on_new_sample(100.0f,79.0f,500UL,1)>78.0f);
    CHECK(example_tick(1501UL,1)==0.0f&&example_mode==EXAMPLE_OFF);
    CHECK(example_pid_cfg.kp==10.0f);
    candidate_fixture();example_begin_trial(100.0f,79.0f,0UL,1);
    CHECK(example_on_new_sample(103.0f,79.0f,500UL,1)==0.0f&&example_mode==EXAMPLE_OFF);
    CHECK(example_pid_cfg.kp==10.0f);
    candidate_fixture();example_begin_trial(100.0f,79.0f,0UL,1);
    example_validation.cfg.warmup_ms=0UL;example_validation.cfg.window_ms=2000UL;
    for(t=500UL;t<=2000UL;t+=500UL)example_on_new_sample(100.0f,79.0f,t,1);
    CHECK(example_mode==EXAMPLE_ACCEPTED);
    CHECK(example_tick(3001UL,1)==0.0f&&example_mode==EXAMPLE_OFF);
    CHECK(example_pid_cfg.kp==10.0f);
    printf("Integration tests: %d passed, %d failed\n",good,bad);
    return bad?1:0;
}

#include <stdio.h>
#include <math.h>
/* 主机编译时用-include强制包含实际验收头，运行真实C代码。 */
static int failed=0,passed=0;
#define CHECK(x) do { if(!(x)){printf("FAIL line %d\n",__LINE__);failed++;}else passed++; } while(0)
static PortableTuneValidationConfig config(void)
{
    PortableTuneValidationConfig c;
    c.target=100.0f;c.tolerance=0.5f;c.minimum_inside_fraction=1.0f;
    c.maximum_peak_to_peak=1.0f;c.maximum_mean_error=0.3f;c.maximum_overshoot=2.0f;
    c.warmup_ms=1000UL;c.window_ms=2000UL;c.max_gap_ms=500UL;
    return c;
}
int main(void)
{
    PortableTuneValidation a,b;
    PortableTuneValidationConfig c;
    unsigned long t;
    c=config(); CHECK(portable_validation_start(&a,&c,0UL));
    for(t=0UL;t<=3000UL;t+=100UL) portable_validation_update(&a,100.0f,t,1);
    CHECK(a.state==PORTABLE_VALIDATION_PASS&&a.count==21UL&&a.mean==100.0f);
    portable_validation_tick(&a,3100UL,0);CHECK(a.state==PORTABLE_VALIDATION_FAIL);
    portable_validation_start(&a,&c,0UL);
    portable_validation_update(&a,102.1f,0UL,1);CHECK(a.state==PORTABLE_VALIDATION_FAIL&&a.reason==6);
    portable_validation_start(&a,&c,0UL);
    portable_validation_tick(&a,501UL,1);CHECK(a.state==PORTABLE_VALIDATION_FAIL&&a.reason==4);
    portable_validation_start(&a,&c,1000UL);
    portable_validation_tick(&a,999UL,1);CHECK(a.state==PORTABLE_VALIDATION_FAIL&&a.reason==3);
    portable_validation_start(&a,&c,0xFFFFFF00UL);
    for(t=0UL;t<=3000UL;t+=100UL)
        portable_validation_update(&a,100.0f,(0xFFFFFF00UL+t)&0xFFFFFFFFUL,1);
    CHECK(a.state==PORTABLE_VALIDATION_PASS);
    portable_validation_start(&a,&c,0UL);
    for(t=0UL;t<=3000UL;t+=100UL) portable_validation_update(&a,100.4f,t,1);
    CHECK(a.state==PORTABLE_VALIDATION_FAIL&&a.reason==7);
    portable_validation_start(&a,&c,0UL);
    for(t=0UL;t<=3000UL;t+=100UL)
        portable_validation_update(&a,(t/100UL)%2?99.4f:100.4f,t,1);
    CHECK(a.state==PORTABLE_VALIDATION_FAIL);
    portable_validation_start(&a,&c,0UL);
    portable_validation_update(&a,100.0f,0UL,1);
    portable_validation_update(&a,100.0f,0UL,1);CHECK(a.count==0UL);
    portable_validation_start(&b,&c,0UL);CHECK(b.state==PORTABLE_VALIDATION_RUNNING&&a.count==0UL);
    c.tolerance=-1.0f;CHECK(!portable_validation_start(&a,&c,0UL));
    c=config();c.maximum_mean_error=1.0f;c.warmup_ms=0UL;
    portable_validation_start(&a,&c,0UL);
    for(t=0UL;t<=2000UL;t+=100UL)
        portable_validation_update(&a,100.0f+0.2f*(float)sin((double)t/500.0),t,1);
    CHECK(a.state==PORTABLE_VALIDATION_PASS&&a.maximum-a.minimum<0.5f);
    printf("Validation tests: %d passed, %d failed\n",passed,failed);
    return failed?1:0;
}

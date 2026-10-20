#define PORTABLE_TUNE_IMPLEMENTATION
#include "../通用自整定.h"
#define PORTABLE_TUNE_VALIDATION_IMPLEMENTATION
#include "../通用自整定验收.h"
#if defined(__C51__)
#define TEST_RAM xdata
#else
#define TEST_RAM
#endif
static PortableTune TEST_RAM tuner;
static PortableTuneConfig TEST_RAM cfg;
void main(void) {
    portable_tune_reset(&tuner);
    portable_tune_defaults(&cfg);
    portable_tune_start(&tuner,&cfg,0UL);
    portable_tune_update(&tuner,100.0f,79.0f,0UL,1);
    portable_tune_cancel(&tuner);
}

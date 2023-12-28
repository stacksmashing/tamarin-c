#include "tamarin_hw.h"
/* Needed by FUSB302.c */
static void platform_usleep(uint64_t us)
{
	sleep_ms(us / 1000);
	sleep_us(us % 1000);
}


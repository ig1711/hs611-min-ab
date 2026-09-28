/*
 * GD32F3x0 peripheral library selection (copied from the vendor
 * Template/gd32f3x0_libopt.h, V2.6.0). Delete entries you do not use to
 * shrink compile time. gd32f3x0.h includes this file when
 * USE_STDPERIPH_DRIVER is defined.
 */

#ifndef GD32F3X0_LIBOPT_H
#define GD32F3X0_LIBOPT_H

#include "gd32f3x0_adc.h"
#include "gd32f3x0_crc.h"
#include "gd32f3x0_ctc.h"
#include "gd32f3x0_dbg.h"
#include "gd32f3x0_dma.h"
#include "gd32f3x0_exti.h"
#include "gd32f3x0_fmc.h"
#include "gd32f3x0_gpio.h"
#include "gd32f3x0_syscfg.h"
#include "gd32f3x0_i2c.h"
#include "gd32f3x0_fwdgt.h"
#include "gd32f3x0_pmu.h"
#include "gd32f3x0_rcu.h"
#include "gd32f3x0_rtc.h"
#include "gd32f3x0_spi.h"
#include "gd32f3x0_timer.h"
#include "gd32f3x0_usart.h"
#include "gd32f3x0_wwdgt.h"
#include "gd32f3x0_misc.h"

#if (defined(GD32F350) || defined(GD32F355) || defined(GD32F370))
#include "gd32f3x0_tsi.h"
#include "gd32f3x0_cec.h"
#include "gd32f3x0_cmp.h"
#include "gd32f3x0_dac.h"
#endif /* GD32F350, GD32F355 and GD32F370 */

#endif /* GD32F3X0_LIBOPT_H */

#ifndef __HRT_H__
#define __HRT_H__


#include <board_config.h>

#include "hardware/esp32_soc.h"
#include "hardware/esp32_tim.h"

#ifdef HRT_TIMER
#define ESP32_HRT_TIMER HRT_TIMER
#define ESP32_HRT_TIMER_PRESCALER (APB_CLK_FREQ / (1000 * 1000))
/* HRT configuration */
#if   HRT_TIMER == 0
# define HRT_TIMER_BASE		0x3FF5F000
# if CONFIG_ESP32_TIM1
#  error must not set CONFIG_ESP32_TIM0=y and HRT_TIMER=0
# endif
#elif HRT_TIMER == 1
# define HRT_TIMER_BASE		0x3FF5F024
# if CONFIG_ESP32_TIM2
#  error must not set CONFIG_ESP32_TIM1=y and HRT_TIMER=1
# endif
#elif HRT_TIMER == 2
# define HRT_TIMER_BASE		0x3FF60000
# if CONFIG_ESP32_TIM2
#  error must not set CONFIG_ESP32_TIM2=y and HRT_TIMER=2
# endif
#elif HRT_TIMER == 3
# define HRT_TIMER_BASE		0x3FF60024
# if CONFIG_ESP32_TIM3
#  error must not set CONFIG_ESP32_TIM3=y and HRT_TIMER=3
# endif
#else
# error HRT_TIMER must be a value between 0 and 3
#endif



#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))


#define rLO 		REG(TIM_LO_OFFSET)
#define rHI 		REG(TIM_HI_OFFSET)
#define rUPDATE 	REG(TIM_UPDATE_OFFSET)
#define rALARMLO 	REG(TIMG_ALARM_LO_OFFSET)
#define rALARMHI 	REG(TIMG_ALARM_HI_OFFSET)

#endif
#endif


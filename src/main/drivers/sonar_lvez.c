#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"
#include "common/maths.h"

#include "drivers/sonar_hcsr04.h"           // FIXME Copy sonarHardware_t or put it elsewhere

#if defined(SONAR) && defined(SONAR_LVEZ)

//static volatile int32_t measurement = -1;
static sonarHardware_t const *sonarHardware;

#define NSAMPLES 3
static int32_t measurements[NSAMPLES];
static volatile int32_t nextMeasurementIdx = 0;

static void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t timing_stop;
    
    if (digitalIn(GPIOB, sonarHardware->echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            //measurement = timing_stop - timing_start;
            measurements[nextMeasurementIdx++] = timing_stop - timing_start;
            if (nextMeasurementIdx >= NSAMPLES)
                nextMeasurementIdx = 0;
        }
    }
    
    EXTI_ClearITPendingBit(sonarHardware->exti_line);
}

void EXTI0_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void lvez_init(const sonarHardware_t *initialSonarHardware)
{
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;
    
    sonarHardware = initialSonarHardware;
    
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif
    
#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif
    
    // echo pin
    gpio.pin = sonarHardware->echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOB, &gpio);
    
#ifdef STM32F10X
    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonarHardware->exti_pin_source);
#endif
    
#ifdef STM32F303xC
    gpioExtiLineConfig(EXTI_PortSourceGPIOB, sonarHardware->exti_pin_source);
#endif
    
    EXTI_ClearITPendingBit(sonarHardware->exti_line);
    
    EXTIInit.EXTI_Line = sonarHardware->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = sonarHardware->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * Get the distance that was measured by the last pulse, in centimeters. When the ground is too far away to be
 * reliably read by the sonar, -1 is returned instead.
 */
int32_t lvez_get_distance(void)
{
    // Do a median filtering
    int32_t measurement = quickMedianFilter3(measurements);

    // Maxbotix LV-EZ datasheet: "This pin outputs a pulse width representation of range. The distance can be calculated using the scale factor of
    // 147uS per inch."
    // So that's 58us per cm
    int32_t distance = measurement / 58;
    
    // this sonar range is up to 6.5m, but let's cut at 6m so we know when its max range is reached.
    if (distance > 600)
        distance = -1;
    
    return distance;
}
#endif


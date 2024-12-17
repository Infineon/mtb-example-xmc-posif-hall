/*******************************************************************************
* File Name:   main.c
*
* Description: This example demonstrates a Position Interface (POSIF) module
*              in Hall sensor mode and uses the CCU40 module to determine the
*              speed of rotation of the motor.
*              Instead of hall motor, the example demonstrates the use of
*              POSIF_HALL module, using simulation via 3 PWM signals.
*
* Related Document: See README.md
*
********************************************************************************
*
* Copyright (c) 2022, Infineon Technologies AG
* All rights reserved.
*
* Boost Software License - Version 1.0 - August 17th, 2003
* Permission is hereby granted, free of charge, to any person or organization
* obtaining a copy of the software and accompanying documentation covered by
* this license (the "Software") to use, reproduce, display, distribute,
* execute, and transmit the Software, and to prepare derivative works of the
* Software, and to permit third-parties to whom the Software is furnished to
* do so, all subject to the following:
*
* The copyright notices in the Software and this entire statement, including
* the above license grant, this restriction and the following disclaimer,
* must be included in all copies of the Software, in whole or in part, and
* all derivative works of the Software, unless such copies or derivative
* works are solely in the form of machine-executable object code generatd by
* a source language processor.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
* SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
* FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
*
*******************************************************************************/

#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"
#include <stdio.h>

/*******************************************************************************
*  Macros
*******************************************************************************/
#define TICKS_PER_SECOND                    (1000U)
#define TICKS_WAIT                          (100U)

/* Define macro to enable/disable printing of debug messages */
#define ENABLE_XMC_DEBUG_PRINT              (0)

/* Define macro to set the loop count before printing debug messages */
#if ENABLE_XMC_DEBUG_PRINT
#define DEBUG_LOOP_COUNT_MAX                (3U)
#endif

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Correct hall event and wrong hall event flag variables */
uint8_t che_flag = 0, whe_flag = 0;

/* CCU8 pulse counter */
uint8_t ccu8_pulse_counter = 0;

/* Timers flag */
bool timers_started = false;

/* Hall input array */
uint8_t hall[3] = {0,0,0};

/* Hall position variable */
uint8_t hall_position = 0;

/* Correct hall event variable */
unsigned long hall_events_interval = 0;

#if ENABLE_XMC_DEBUG_PRINT
/* Initialize the current loop count to zero */
static uint32_t debug_loop_count = 0;
#endif

 /*******************************************************************************
 * Function Name: SysTick Handler
 ********************************************************************************
 * Summary:
 *  This is the interrupt handler function for the System Tick interrupt. This
 *  function print the time interval between two correct hall events and wrong
 *  hall event.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void SysTick_Handler(void)
{
    /* Ticks wait */
    static uint32_t ticks = 0;

    ticks++;

    /* Wait for 500ms delay */
    if (ticks == TICKS_WAIT)
    {
        ticks = 0;
        /* Check if correct hall event occurs */
        if((che_flag == 1) && (whe_flag == 0))
        {
            /* Set che_flag to 0 */
            che_flag = 0;
            #if ENABLE_XMC_DEBUG_PRINT
                debug_loop_count++;
                if (debug_loop_count == DEBUG_LOOP_COUNT_MAX)
                    printf("All three correct hall events occurs\r\n");
            #else
                /* Print the time interval between two correct hall events in nano seconds */
                printf("Time interval between two correct hall events: %luns\r\n", hall_events_interval);
            #endif
        }
        /* Check if wrong hall event occurs */
        else if((che_flag == 0) && (whe_flag == 1))
        {
            /* Set whe_flag to 0 */
            whe_flag = 0;
            /* Print the wrong hall event */
            printf("Wrong hall event\r\n");
        }
    }
}

/*******************************************************************************
* Function Name: POSIF0_0_IRQHandler
********************************************************************************
* Summary:
*  POSIF0_0_IRQHandler interrupt handler function will occur for every
*  correct hall pattern. Calculate the timing between two correct hall events.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void POSIF0_0_IRQHandler(void)
{
    /* Get the capture timer value */
    uint16_t captured_value = 0;

    /* Set che_flag to 1 */
    che_flag = 1;
    /* Set whe_flag to 0 */
    whe_flag = 0;

    /* Check for a rising edge of POSIF0.OUT1 signal */
    if (XMC_CCU4_SLICE_GetEvent(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_IRQ_ID_EVENT0))
    {
        /* Clear event*/
        XMC_CCU4_SLICE_ClearEvent(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_IRQ_ID_EVENT0);

        /* Get captured timer value on rising edge */
        captured_value = XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW, 1U);

        /* Calculate the time between two correct hall events
         * (captured_value * prescaler * 1000) / clock */
        hall_events_interval = captured_value * HALL_SPEED_TIMER_TICK_NS;
    }
    /* Clear pending event */
    XMC_POSIF_ClearEvent(HALL_POSIF_HW, XMC_POSIF_IRQ_EVENT_CHE);
}

/*******************************************************************************
* Function Name: POSIF0_1_IRQHandler
********************************************************************************
* Summary:
*  POSIF0_1_IRQHandler interrupt handler function will occur for every
*  wrong hall pattern.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void POSIF0_1_IRQHandler(void)
{
    /* Set whe_flag to 1 */
    whe_flag = 1;
    /* Set che_flag to 0 */
    che_flag = 0;

    /* Clear pending event */
    XMC_POSIF_ClearEvent(HALL_POSIF_HW, XMC_POSIF_IRQ_EVENT_WHE);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It starts the POSIF Module in hall mode and uses
*  the CCU40 module to determine the speed of rotation of the motor. Each time a
*  correct Hall event is detected, an interrupt is generated. Timing between the
*  two correct hall events are displayed on the terminal. Each time a wrong hall
*  event is detected, an interrupt is generated and displayed on the terminal.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);

    #if ENABLE_XMC_DEBUG_PRINT
    printf("Initialization done\r\n");
    #else
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("============================================================ \r\n");
    printf("XMC MCU: POSIF Hall example \r\n");
    printf("============================================================ \r\n");
    #endif


    /* Set priority */
    NVIC_SetPriority(POSIF0_0_IRQn, 0U);
    NVIC_SetPriority(POSIF0_1_IRQn, 1U);

    /* Enable IRQ */
    NVIC_EnableIRQ(POSIF0_0_IRQn);
    NVIC_EnableIRQ(POSIF0_1_IRQn);

    /* Print the CHE/WHE occurrence for every 500ms */
    SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);

    /* Start HALL_1, HALL_2 and HALL_3 Timers */
    XMC_CCU8_SLICE_StartTimer(HALL_1_HW);
    XMC_CCU8_SLICE_StartTimer(HALL_2_HW);
    XMC_CCU8_SLICE_StartTimer(HALL_3_HW);

    while (1)
    {
        XMC_Delay(1);
        /* Checks if period match event has occurred */
        if (XMC_CCU8_SLICE_GetEvent(HALL_3_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH))
        {
            /* Timers are not started and CCU8 pulse counter greater than 3 */
            if ((ccu8_pulse_counter++ > 3) && (!timers_started))
            {
                /* Start the Encoder */
                XMC_POSIF_Start(HALL_POSIF_HW);

                /* Read the Hall input GPIO pins */
                hall[0] = XMC_GPIO_GetInput(HALL_INPUT_1_PORT, HALL_INPUT_1_PIN);
                hall[1] = XMC_GPIO_GetInput(HALL_INPUT_2_PORT, HALL_INPUT_2_PIN);
                hall[2] = XMC_GPIO_GetInput(HALL_INPUT_3_PORT, HALL_INPUT_3_PIN);
                hall_position = (uint8_t)((hall[0] | (hall[1] << 1) | (hall[2] << 2)));

                /* Configure current and expected hall patterns */
                XMC_POSIF_HSC_SetHallPatterns(HALL_POSIF_HW, HALL_POSIF_Hall_Pattern[hall_position ? hall_position : 1]);

                /* Update hall pattern */
                XMC_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW);

                /* Start CCU4 timers */
                XMC_CCU4_SLICE_StartTimer(HALL_DELAY_TIMER_HW);
                XMC_CCU4_SLICE_StartTimer(HALL_SPEED_TIMER_HW);

                /* Sets the timers flag to the true value */
                timers_started = true;
            }
            XMC_CCU8_SLICE_ClearEvent(HALL_3_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
        }

        /* Delay and Speed timers are started */
        if (timers_started)
        {
            /* Read the Hall input GPIO pins */
            hall[0] = XMC_GPIO_GetInput(HALL_INPUT_1_PORT, HALL_INPUT_1_PIN);
            hall[1] = XMC_GPIO_GetInput(HALL_INPUT_2_PORT, HALL_INPUT_2_PIN);
            hall[2] = XMC_GPIO_GetInput(HALL_INPUT_3_PORT, HALL_INPUT_3_PIN);
            hall_position = (uint8_t)((hall[0] | (hall[1] << 1) | (hall[2] << 2)));

            /* Configure current and expected hall patterns */
            XMC_POSIF_HSC_SetHallPatterns(HALL_POSIF_HW, HALL_POSIF_Hall_Pattern[hall_position ? hall_position : 1]);

            /* Update hall pattern */
            XMC_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW);
        }
    }
}

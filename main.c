/*******************************************************************************
* File Name:   main.c
*
* Description: This example demonstrates a Position Interface (POSIF) module in
*              Hall sensor mode and uses the CCU40 module to determine the
*              speed of rotation of the motor.
*
* Related Document: See README.md
*
********************************************************************************
*
* Copyright (c) 2020, Infineon Technologies AG
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
#include "xmc_posif.h"
#include "retarget_io.h"
#include "stdio.h"
#include "xmc_ccu4.h"

/*******************************************************************************
*  Macros
*******************************************************************************/
/* Define macros for XMC1400 Boot kit */
#ifdef TARGET_KIT_XMC14_BOOT_001
#define INPUT_0         (XMC_POSIF_INPUT_PORT_B)     /* Choice of INPUT_PORT_B */
#define INPUT_1         (XMC_POSIF_INPUT_PORT_A)     /* Choice of INPUT_PORT_A */
#define INPUT_2         (XMC_POSIF_INPUT_PORT_A)     /* Choice of INPUT_PORT_A */
#define INPUT_MAPPED_1  (XMC_CCU4_SLICE_INPUT_AE)    /* Port E */
#define INPUT_MAPPED_2  (XMC_CCU4_SLICE_INPUT_AF)    /* Port F */
#endif

/* Define macros for XMC4700 Relax kit */
#ifdef TARGET_KIT_XMC47_RELAX_V1
#define INPUT_0         (XMC_POSIF_INPUT_PORT_B)     /* Choice of INPUT_PORT_B */
#define INPUT_1         (XMC_POSIF_INPUT_PORT_B)     /* Choice of INPUT_PORT_B */
#define INPUT_2         (XMC_POSIF_INPUT_PORT_B)     /* Choice of INPUT_PORT_A */
#define INPUT_MAPPED_1  (XMC_CCU4_SLICE_INPUT_E)     /* Port E */
#define INPUT_MAPPED_2  (XMC_CCU4_SLICE_INPUT_F)     /* Port F */
#endif

/* This gives the current and expected hall pattern in a register format */
#define HALL_POSIF_MCM(EP,CP) (((uint32_t)EP<< 3)|(uint32_t)CP)
#define TICKS_PER_SECOND      (1000U)
#define TICKS_WAIT            (500U)

/*******************************************************************************
* Data Structures
*******************************************************************************/
/* POSIF module configuration */
XMC_POSIF_CONFIG_t posif_config =
{
    .mode   = XMC_POSIF_MODE_HALL_SENSOR,  /* POSIF Operational mode */
    .input0 = INPUT_0,                     /* Choice of input for Input-0 */
    .input1 = INPUT_1,                     /* Choice of input for Input-1 */
    .input2 = INPUT_2,                     /* Choice of input for Input-2 */
    .filter = 0,                           /* Input filter configuration */
};

/* POSIF hall sensor configuration */
XMC_POSIF_HSC_CONFIG_t posif_hall_config =
{
    .disable_idle_signal   = 1,  /* Disable idle signal upon wrong hall event */
    .sampling_trigger      = 0,  /* HSDA is used to trigger POSIF to sample hall pattern */
    .sampling_trigger_edge = 0   /* Rising edge */
};

/* Event 0: Start the slice on rising edge of POSIF0.OUT0 */
XMC_CCU4_SLICE_EVENT_CONFIG_t start_event0_config =
{
    .mapped_input = INPUT_MAPPED_1,                                     /* Input signal for the Event */
    .edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,  /* Rising edge */
    .level        = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH, /* Active high */
    .duration     = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED                /* Filter disabled */
};

/* Event 0: Capture on rising edge of POSIF0.OUT1 */
XMC_CCU4_SLICE_EVENT_CONFIG_t capture_event0_config =
{
    .mapped_input = INPUT_MAPPED_2,                                     /* Input signal for the Event */
    .edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,  /* Rising edge */
    .level        = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH, /* Active high */
    .duration     = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED                /* Filter disabled */
};

/*******************************************************************************
* Global variables
*******************************************************************************/
/* This saves the configured pattern in clockwise direction.
 * Each value of this hall_pattern table is equal to
 * (expected hall << 3 | current hall)
 */
uint8_t hall_pattern[] =
{
    (uint8_t)HALL_POSIF_MCM(0,0),(uint8_t)HALL_POSIF_MCM(3,1),
    (uint8_t)HALL_POSIF_MCM(6,2),(uint8_t)HALL_POSIF_MCM(2,3),
    (uint8_t)HALL_POSIF_MCM(5,4),(uint8_t)HALL_POSIF_MCM(1,5),
    (uint8_t)HALL_POSIF_MCM(4,6),(uint8_t)HALL_POSIF_MCM(0,0)
};

/* Clock variable */
static uint32_t clock = 0;

/* Prescaler variable */
static uint32_t prescaler = 0;

/* Correct hall event and wrong hall event flag variables */
uint8_t che_flag = 0, whe_flag =0;

/* Hall input array */
uint8_t hall[3] = {0,0,0};

/* Hall position variable */
uint8_t hall_position = 0;

/* Correct hall event variable */
uint32_t correct_hall_event = 0;

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
            /* Print the time interval between two correct hall events in nano seconds */
            printf("Time interval between two correct hall events: %luns\r\n", correct_hall_event);
        }
        /* Check if wrong hall event occurs */
        else if((che_flag == 0) && (whe_flag == 1))
        {
            /* Set whe_flag to 0 */
            whe_flag = 0;
            /* Print the wrong hall event */
            printf("Wrong Hall Event \r\n");
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
    uint16_t capture_value = 0;

    /* Set che_flag to 1 */
    che_flag = 1;
    /* Set whe_flag to 0 */
    whe_flag = 0;

    /* Check for a rising edge of POSIF0.OUT1 signal */
    if (XMC_CCU4_SLICE_GetEvent(CAPTURE_0_HW, XMC_CCU4_SLICE_IRQ_ID_EVENT0))
    {
        /* Clear event*/
        XMC_CCU4_SLICE_ClearEvent(CAPTURE_0_HW, XMC_CCU4_SLICE_IRQ_ID_EVENT0);

        /* Get captured timer value on rising edge */
        capture_value = XMC_CCU4_SLICE_GetCaptureRegisterValue(CAPTURE_0_HW,1U);

        /* Calculate the time between two correct hall events */
        correct_hall_event = (uint32_t)((capture_value * prescaler * 1000)/clock);
    }
    /* Clear pending event */
    XMC_POSIF_ClearEvent(POSIF0, XMC_POSIF_IRQ_EVENT_CHE);
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
    XMC_POSIF_ClearEvent(POSIF0, XMC_POSIF_IRQ_EVENT_WHE);
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

    #ifdef TARGET_KIT_XMC14_BOOT_001
    /* Get CCU clock frequency in Hertz */
    clock = (XMC_SCU_CLOCK_GetFastPeripheralClockFrequency()/1000000);
    #endif
    #ifdef TARGET_KIT_XMC47_RELAX_V1
    /* Get CCU clock frequency in Hertz */
    clock = (XMC_SCU_CLOCK_GetCcuClockFrequency()/1000000);
    #endif

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    retarget_io_init();

    /* Get the prescaler value used in CCU40_CC41 Slice */
    prescaler = (1 << (XMC_CCU4_SLICE_PRESCALER_t)XMC_CCU4_SLICE_GetPrescaler(CAPTURE_0_HW));

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("============================================================ \r\n");
    printf("XMC MCU: POSIF Hall example \r\n");
    printf("============================================================ \r\n");

    /* Configure event for CCU40_CC40 and CCU40_CC41 Slices */
    XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC40, XMC_CCU4_SLICE_EVENT_0, &start_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0, &capture_event0_config);

    /* Initialize POSIF module */
    XMC_POSIF_Init(POSIF0, &posif_config);

    /* Initialize hall sensor mode */
    XMC_POSIF_HSC_Init(POSIF0, &posif_hall_config);

    /* Enables event generation */
    XMC_POSIF_EnableEvent(POSIF0, XMC_POSIF_IRQ_EVENT_CHE);
    XMC_POSIF_EnableEvent(POSIF0, XMC_POSIF_IRQ_EVENT_WHE);

    /* Connect correct hall event to SR0 and wrong hall event to SR1 */
    XMC_POSIF_SetInterruptNode(POSIF0, XMC_POSIF_IRQ_EVENT_CHE, XMC_POSIF_SR_ID_0);
    XMC_POSIF_SetInterruptNode(POSIF0, XMC_POSIF_IRQ_EVENT_WHE, XMC_POSIF_SR_ID_1);

    /* Set priority */
    NVIC_SetPriority(POSIF0_0_IRQn, 0U);
    NVIC_SetPriority(POSIF0_1_IRQn, 1U);

    /* Enable IRQ */
    NVIC_EnableIRQ(POSIF0_0_IRQn);
    NVIC_EnableIRQ(POSIF0_1_IRQn);

    /* Start the POSIF module */
    XMC_POSIF_Start(POSIF0);

    /* Start CCU40_CC40 and CCU40_CC41 Timers */
    XMC_CCU4_SLICE_StartTimer(DELAY_0_HW);
    XMC_CCU4_SLICE_StartTimer(CAPTURE_0_HW);

    /* Print the CHE/WHE occurrence for every 500ms */
    SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);

    while (1)
    {
        /* Read the Hall input GPIO pins */
        hall[0]=XMC_GPIO_GetInput(HALL_INPUT_1_PORT, HALL_INPUT_1_PIN);
        hall[1]=XMC_GPIO_GetInput(HALL_INPUT_2_PORT, HALL_INPUT_2_PIN);
        hall[2]=XMC_GPIO_GetInput(HALL_INPUT_3_PORT, HALL_INPUT_3_PIN);
        hall_position = (uint8_t)((hall[0] | (hall[1] << 1) | (hall[2] << 2)));

        /* Configure current and expected hall patterns */
        XMC_POSIF_HSC_SetHallPatterns(POSIF0, hall_pattern[hall_position]);
        /* Update hall pattern */
        XMC_POSIF_HSC_UpdateHallPattern(POSIF0);
    }
}
/* [] END OF FILE */

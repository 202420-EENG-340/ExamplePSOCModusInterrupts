/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#if defined (CY_USING_HAL)
#include "cyhal.h"
#endif
#include "cybsp.h"
#include "cy_sysint.h"
#include "cy_tcpwm.h"
#include "cycfg.h"
/******************************************************************************
* Macros
*******************************************************************************/

#define MY_TCPWM_CNT_NUM   (0UL)


/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile int val = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

void Interrupt_Handler_Port0 (void){
    Cy_TCPWM_ClearInterrupt(TCPWM0, MY_TCPWM_CNT_NUM, CY_TCPWM_INT_ON_CC );

    val = !val;
    Cy_GPIO_Write(CYBSP_LED3_PORT, CYBSP_LED3_PIN, val);
}
    
/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
*********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Scenario: Vector table is relocated to RAM in __RAM_VECTOR_TABLE[] */
    /* Prototype of ISR function for port interrupt 0 */
    cy_stc_sysint_t intrCfg =
    {
    /*.intrsrc=*/ tcpwm_0_cnt_0_IRQ, /* Interrupt source is PWM counter 0 interrupt */
    /*.intrPriority =*/ 3UL /* Interrupt priority is 3 */
    };
    /* Initialize the interrupt with vector at Interrupt_Handler_Port0() */
    Cy_SysInt_Init(&intrCfg, &Interrupt_Handler_Port0);
    /* Enable the interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);
   
    

#if defined (CY_DEVICE_SECURE) && defined (CY_USING_HAL)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_TCPWM_PWM_Init(TCPWM0, MY_TCPWM_CNT_NUM, &tcpwm_0_cnt_0_config);
    Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_CNT_NUM);
    Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_CNT_NUM);
    
    /* Enable global interrupts */
    __enable_irq();
    
    for (;;)
    {
    //     __disable_irq();
    //     currVal = val;
    //     __enable_irq();
    //     if (currVal != lastVal)
    //     {
    //         Cy_GPIO_Write(CYBSP_LED3_PORT, CYBSP_LED3_PIN, currVal);
    //         lastVal = currVal;
    //     }
    }
}

/* [] END OF FILE */

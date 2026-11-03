/******************************************************************************
* File Name:   main.c
*
* Description:
*  This file provides reference usage of SCB-UART self tests for  PSoC Control 
*  MCU.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "SelfTest.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define MAX_INDEX_VAL 0xFFF0u
#define CONFIG_ALL_DELAY 100u

#define INTR_SRC_MASTER CYBSP_DUT_UART_MASTER_IRQ
#define INTR_SRC_SLAVE  CYBSP_DUT_UART_SLAVE_IRQ
#define INTR_SRC_TIMER  CYBSP_TIMER_UART_MASTER_IRQ

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_uart_context_t CYBSP_UART_context;
cy_en_scb_uart_status_t initstatus;
/* Debug UART context */
cy_stc_scb_uart_context_t  DEBUG_UART_context;
/* Debug UART HAL object */
mtb_hal_uart_t DEBUG_UART_hal_obj;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void Protocol_Test_UART_Init(void);
static void Timeout_Counter_Init(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It configures one SCB instance as an UART and also 
* a Smart I/O™ to internally connect the TX and RX pins. The function validates 
* bidirectional communication within the UART interface by transmitting data 
* via TX and receiving it back through RX.
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

    /* Uart test data */
    uint8_t rxd[RX_BUFF_SIZE];
    uint8_t txd[] = "1234567890ABCDEF";
    uint16_t count = 0u;
    uint8_t slave_resp_res;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    init_cycfg_all();
    
    Cy_SysLib_Delay(CONFIG_ALL_DELAY);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    if (result != CY_RSLT_SUCCESS)
    {
       CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Initialize HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
       CY_ASSERT(0);
    }
    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Init HW for UART Protocol test and Setup ISRs*/
    Timeout_Counter_Init();
    Protocol_Test_UART_Init();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("Class-B Safety Test: Communication protocol\r\n");

    /* Init UART Slave and Master Protocol Structures and Enable the UART HW */
    UartMesMaster_Init(CYBSP_DUT_UART_MASTER_HW, CYBSP_TIMER_UART_MASTER_HW, CYBSP_TIMER_UART_MASTER_NUM);
    UartMesSlave_Init(CYBSP_DUT_UART_SLAVE_HW, 2u);


    for (;;)
    {
        /*    Master communication process */
        if(UM_BUSY != UartMesMaster_State())
        {

            /* Master is ready for send
             * Show previous communication result */
            if(UM_COMPLETE != UartMesMaster_State())
            {

                /* No respond timeout error */
                if (count > 0u)
                {
                    printf("\r\nCommunication Protocol test: error \r\n");

                    /* For demo purposes in case of error detection
                    * message is printed to UART Debug and code execution
                    * is stopped here in a while loop */
                    while(1u)
                    {
                    }
                }
            }

            printf("\rCommunication Protocol testing at run-time... count = %d",count);

            /* Send new packet */
            UartMesMaster_DataProc(2u, txd, RX_TEST_SIZE, rxd, sizeof(rxd));
         }

         /* Slave communication process */
         if(UM_PACKREADY == UartMesSlave_State())
         {

             /* Slave have a marker
              * analyze received data and prepare respond */
             slave_resp_res = UartMesSlave_Respond((uint8_t *)UartMesSlave_GetDataPtr(), UartMesSlave_GetDataSize());
             if(slave_resp_res)
             {
                 CY_ASSERT(0);
             }
          }

          count++;
          if (count > MAX_INDEX_VAL)
          {
              count = 0u;
          }


          /* Delay emulates user code */
          Cy_SysLib_Delay(50u);
    }

}

/*****************************************************************************
* Function Name: Protocol_Test_UART_Init
******************************************************************************
*
* Summary:
*  Init UART component and hook appropriate ISRs for the background protocol
*  routine.
*
* Parameters:
*  NONE
*
* Return:
*  NONE
******************************************************************************/
static void Protocol_Test_UART_Init(void)
{
    cy_en_sysint_status_t int_result;
    cy_en_scb_uart_status_t uart_result;
    cy_stc_sysint_t intrCfg =
    {
       /*.intrSrc =*/ INTR_SRC_MASTER, /* Interrupt source is UART interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };
    int_result = Cy_SysInt_Init(&intrCfg, UartMesMaster_Msg_ISR);
    if(int_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0u);
    }

    /* Enable Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Reuse intrCfg Struct for Slave UART */
    intrCfg.intrSrc = INTR_SRC_SLAVE; /* Interrupt source is UART interrupt */
    int_result = Cy_SysInt_Init(&intrCfg, UartMesSlave_Msg_ISR);
    if(int_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0u);
    }
    /* Enable Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Initialize the Master UART */
    uart_result = Cy_SCB_UART_Init(CYBSP_DUT_UART_MASTER_HW, &CYBSP_DUT_UART_MASTER_config, NULL);
    if(uart_result != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0u);
    }

    /* Initialize the Slave UART */
    uart_result = Cy_SCB_UART_Init(CYBSP_DUT_UART_SLAVE_HW, &CYBSP_DUT_UART_SLAVE_config, NULL);
    if(uart_result != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0u);
    }
}

/******************************************************************************
* Function Name: Timeout_Counter_Init
*******************************************************************************
*
* Summary:
* Initialize the Timer interrupt for the UART protocol timeout.
*
* Parameters:
*  NONE
*
* Return:
*  NONE
******************************************************************************/
static void Timeout_Counter_Init(void)
{
    cy_en_sysint_status_t int_result;
    cy_en_tcpwm_status_t tcpwm_result;

    cy_stc_sysint_t intrCfg =
    {
       /*.intrSrc =*/ INTR_SRC_TIMER, /* Interrupt source is Timer interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };

    int_result = Cy_SysInt_Init(&intrCfg, UartMesMaster_Timeout_ISR);

    if(int_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Init and Enable timer */
    tcpwm_result = Cy_TCPWM_Counter_Init(CYBSP_TIMER_UART_MASTER_HW, CYBSP_TIMER_UART_MASTER_NUM, &CYBSP_TIMER_UART_MASTER_config);
    if(tcpwm_result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_UART_MASTER_HW, CYBSP_TIMER_UART_MASTER_NUM);
}
/* [] END OF FILE */

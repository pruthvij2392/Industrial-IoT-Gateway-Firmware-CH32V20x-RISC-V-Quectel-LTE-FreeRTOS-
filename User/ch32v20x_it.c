/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"

void NMI_Handler(void) __attribute__((interrupt()));
void HardFault_Handler(void) __attribute__((interrupt()));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   Handles Non-Maskable Interrupt (NMI) exceptions.
 *          In embedded RISC-V systems, NMIs can be triggered by hardware clock failures
 *          or critical bus parity errors. Enters an infinite trap loop.
 *
 * @return  none
 */
void NMI_Handler(void)
{
    while(1)
    {

    }
}

/*******************************************************************************
 * @fn        HardFault_Handler
 *
 * @brief     Handles RISC-V Hard Fault exceptions (e.g. invalid memory access,
 *            unaligned address faults, illegal instructions, or stack overflows).
 * 
 * @note      In an unattended industrial IoT gateway, stopping execution in a loop
 *            would cause permanent field bricking. Calling `NVIC_SystemReset()` performs
 *            an immediate hardware system reset to recover normal gateway operations.
 *
 * @return    None
 */
void HardFault_Handler(void)
{
  NVIC_SystemReset(); /* Force system reboot on fatal exception for continuous field availability */
  while (1)
  {
  }
}



/*******************************************************************************
* File Name: Direction_Flag.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "Direction_Flag.h"

static Direction_Flag_BACKUP_STRUCT  Direction_Flag_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: Direction_Flag_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet Direction_Flag_SUT.c usage_Direction_Flag_Sleep_Wakeup
*******************************************************************************/
void Direction_Flag_Sleep(void)
{
    #if defined(Direction_Flag__PC)
        Direction_Flag_backup.pcState = Direction_Flag_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            Direction_Flag_backup.usbState = Direction_Flag_CR1_REG;
            Direction_Flag_USB_POWER_REG |= Direction_Flag_USBIO_ENTER_SLEEP;
            Direction_Flag_CR1_REG &= Direction_Flag_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Direction_Flag__SIO)
        Direction_Flag_backup.sioState = Direction_Flag_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        Direction_Flag_SIO_REG &= (uint32)(~Direction_Flag_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: Direction_Flag_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to Direction_Flag_Sleep() for an example usage.
*******************************************************************************/
void Direction_Flag_Wakeup(void)
{
    #if defined(Direction_Flag__PC)
        Direction_Flag_PC = Direction_Flag_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            Direction_Flag_USB_POWER_REG &= Direction_Flag_USBIO_EXIT_SLEEP_PH1;
            Direction_Flag_CR1_REG = Direction_Flag_backup.usbState;
            Direction_Flag_USB_POWER_REG &= Direction_Flag_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Direction_Flag__SIO)
        Direction_Flag_SIO_REG = Direction_Flag_backup.sioState;
    #endif
}


/* [] END OF FILE */
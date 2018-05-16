/*
 * motor_driver.c
 *
 *  Created on: 10 Feb 2018
 *      Author: AfdhalAtiffTan
 */

#include "motor_driver.h"

//*****************************************************************************
//
//! Simple motor pwm control.
//!
//! \param pwm is the desired compare value. The duty cycle depends on period register.
//!
//! This function sets the active pwm pin by checking the \param pwm sign.
//! The maximum value for pwm is +/- 0xFFFF.
//!
//! \return None.
//
//*****************************************************************************
void bidirectional_motor(signed long pwm)
{
    //sanity check
    if(pwm > 16700000)
        pwm = 16700000;
    if(pwm < -16700000)
        pwm= -16700000;

    //forward
    if(pwm >= 0)
    {
        HRPWM_setCounterCompareValue(EPWM3_BASE,
                                     HRPWM_COUNTER_COMPARE_A,
                                    pwm);
        HRPWM_setCounterCompareValue(EPWM3_BASE,
                                     HRPWM_COUNTER_COMPARE_B,
                                    0);
    }

    //reverse
    else
    {
        pwm *= -1;

        HRPWM_setCounterCompareValue(EPWM3_BASE,
                                     HRPWM_COUNTER_COMPARE_A,
                                    0);
        HRPWM_setCounterCompareValue(EPWM3_BASE,
                                     HRPWM_COUNTER_COMPARE_B,
                                    pwm);
    }
}

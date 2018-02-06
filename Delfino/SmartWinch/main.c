//#############################################################################
//
// FILE:   main.c
//
// TITLE:  Empty Project
//
// Empty Project Example
//
// This example is an empty project setup for Driverlib development.
//
//#############################################################################
// $TI Release: F2837xS Support Library v3.03.00.00 $
// $Release Date: Thu Dec  7 18:53:06 CST 2017 $
// $Copyright:
// Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include <stdio.h>
#include "driverlib.h"
#include "device.h"

//
// Globals
//
char *msg; //for fixed msg
char str[128]; //for runtime msg
uint16_t receivedChar[32]; //for general purpose reception

//temporary pid
volatile float setpoint = 7680.0;
volatile float kp=0.0,ki=0.0,kd=700;
volatile float error=0.0, input_previous=0.0, error_integrated=0.0, input_differentiated=0.0;
volatile float out_min = -35000.0, out_max = 35000.0;
volatile float input=0.0, output=0.0;


void bidirectional_motor(long pwm);

__interrupt void epwm2ISR(void);

//
// epwm2ISR - ePWM 2 ISR
//
__interrupt void epwm2ISR(void)
{
    volatile static int cnt=0;

    //heartbeat on led
    if(cnt++>15)
    {
        cnt=0;
        GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
    }

    //
    // PID here
    //

    input = (float) EQEP_getPositionLatch(EQEP1_BASE);

    //<Proportional>
    error = setpoint - input;
    output = kp*error;
    //</Proportional>

    //<Integration>
    error_integrated += error;

    //preventing integrator windup
    error_integrated = error_integrated > out_max ? out_max : error_integrated;
    error_integrated = error_integrated < out_min ? out_min : error_integrated;

    //reseting integrator
    if(ki == 0.0)
        error_integrated = 0.0;

    output += ki/1000.0*error_integrated; //added divide by 1000 to allow for fractional math
    //</Integration>

    //<Differentiation> (“Derivative on Measurement”)
    input_differentiated = input - input_previous;
    input_previous = input;

    output -= kd * input_differentiated;
    //</Differentiation>

    //output clamp
    output = output > out_max ? out_max : output;
    output = output < out_min ? out_min : output;

    bidirectional_motor((long)output);



    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM2_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//simple motor control
void bidirectional_motor(long pwm)
{
    //sanity check
    if(pwm>65000)
        pwm=65000;
    if(pwm<-65000)
        pwm= -65000;

    if(pwm>=0)
    {
        EPWM_setCounterCompareValue(EPWM2_BASE,
                                    EPWM_COUNTER_COMPARE_A,
                                    pwm);
        EPWM_setCounterCompareValue(EPWM2_BASE,
                                    EPWM_COUNTER_COMPARE_B,
                                    0);
    }
    else
    {
        pwm *= -1;

        EPWM_setCounterCompareValue(EPWM2_BASE,
                                    EPWM_COUNTER_COMPARE_A,
                                    0);
        EPWM_setCounterCompareValue(EPWM2_BASE,
                                    EPWM_COUNTER_COMPARE_B,
                                    pwm);
    }
}


void init_gpio()
{
    //set pin12 and pin13 as gpio
    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED2);

    //set gpio12 and gpio13 as pushpullOut or HiZin
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);

    //turn off both led (active-low)
    GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1);
    GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1);

    //set both pins as output
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);
}

void init_uart()
{
    // GPIO28 is the SCI Rx pin.
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    // GPIO29 is the SCI Tx pin.
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    // Initialize SCIA and its FIFO.
    SCI_performSoftwareReset(SCIA_BASE);

    // Configure SCIA for echoback.
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200, (  SCI_CONFIG_WLEN_8 |
                                                            SCI_CONFIG_STOP_ONE |
                                                            SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
}

void init_eqep()
{
    //set as eqep
    GPIO_setPinConfig(DEVICE_GPIO_CFG_EQEP1A);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_EQEP1B);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_EQEP1I);

    //set HiZin (because the board already has pull-up resistors and an input buffer)
    GPIO_setPadConfig(DEVICE_GPIO_PIN_EQEP1A, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_EQEP1B, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_EQEP1I, GPIO_PIN_TYPE_STD);

    //set both pins as input (maybe not needed?)
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_EQEP1A, GPIO_DIR_MODE_IN);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_EQEP1B, GPIO_DIR_MODE_IN);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_EQEP1I, GPIO_DIR_MODE_IN);


    //
    // Configure the decoder for quadrature count mode
    //
    EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_2X_RESOLUTION |
                                       EQEP_CONFIG_QUADRATURE |
                                       EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(EQEP1_BASE, EQEP_EMULATIONMODE_RUNFREE);

    //
    // Configure the position counter to reset on an index event
    //
    EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_IDX,
                                  0xFFFFFFFF);

    //
    // Enable the unit timer, setting the frequency to 100 Hz
    //
    EQEP_enableUnitTimer(EQEP1_BASE, (DEVICE_SYSCLK_FREQ / 100));

    //
    // Configure the position counter to be latched on a unit time out
    //
    EQEP_setLatchMode(EQEP1_BASE, EQEP_LATCH_UNIT_TIME_OUT);

    //
    // Enable the eQEP1 module
    //
    EQEP_enableModule(EQEP1_BASE);

    //
    // Configure and enable the edge-capture unit. The capture clock divider is
    // SYSCLKOUT/128. The unit-position event divider is QCLK/32.
    //
    //128 because it is the slowest posible, i.e. 20MHz/128
    //32 because 32count per rotation
    EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_128,
                          EQEP_UNIT_POS_EVNT_DIV_32);

    EQEP_enableCapture(EQEP1_BASE);

    //set reset position
    EQEP_setInitialPosition(EQEP1_BASE, 0x00000000);

    //enable index reset
    EQEP_setPositionInitMode(EQEP1_BASE, EQEP_INIT_FALLING_INDEX);
}

void init_pwm()
{
    //
    // Clear Compare values
    //
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                0);
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                EPWM_COUNTER_COMPARE_B,
                                0);


    GPIO_setPinConfig(DEVICE_GPIO_CFG_EPWM2A);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_EPWM2B);

    GPIO_setPadConfig(DEVICE_GPIO_PIN_EPWM2A, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_EPWM2B, GPIO_PIN_TYPE_STD);

    GPIO_writePin(DEVICE_GPIO_PIN_EPWM2A, 0);
    GPIO_writePin(DEVICE_GPIO_PIN_EPWM2B, 0);

    GPIO_setDirectionMode(DEVICE_GPIO_PIN_EPWM2A, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_EPWM2B, GPIO_DIR_MODE_OUT);


    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM2_BASE, 0xFFFF);
    EPWM_setPhaseShift(EPWM2_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);


    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setClockPrescaler(EPWM2_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    //
    // Assign the interrupt service routines to ePWM interrupt
    //
    Interrupt_register(INT_EPWM2, &epwm2ISR);


    //
    // Enable ePWM interrupts
    //
    Interrupt_enable(INT_EPWM2);

    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 8th event
    //
    EPWM_setInterruptSource(EPWM2_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM2_BASE);
    EPWM_setInterruptEventCount(EPWM2_BASE, 8U);
}

void init_smartwinch()
{
    init_gpio();
    init_uart();
    init_eqep();
    init_pwm();
}

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //init required modules
    init_smartwinch();


    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;



    //
    // Send starting message.
    //
    msg = "Heeyaa\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 8);

    //give some time for power supply to settle before powering the motor
    DEVICE_DELAY_US(100000);
    kp=150.0;

    while(1)
    {
        uint32_t temp;

        if((EQEP_getStatus(EQEP1_BASE) & EQEP_STS_UNIT_POS_EVNT) != 0)
        {
            //
            // No capture overflow
            //
            if((EQEP_getStatus(EQEP1_BASE) & EQEP_STS_CAP_OVRFLW_ERROR) == 0)
            {
                temp = EQEP_getCapturePeriodLatch(EQEP1_BASE);


                float freq = (DEVICE_SYSCLK_FREQ / 128) / temp / 2; // 60/120

                temp = (uint32_t) freq;
                uint32_t currentCount = EQEP_getPositionLatch(EQEP1_BASE);


                int len = sprintf(str, "cnt: %lu, rpm: %lu\n\r", currentCount, temp);
                SCI_writeCharArray(SCIA_BASE, (uint16_t*)str, len);
            }

            //
            // Clear unit position event flag and overflow error flag
            //
            EQEP_clearStatus(EQEP1_BASE, (EQEP_STS_UNIT_POS_EVNT |
                                          EQEP_STS_CAP_OVRFLW_ERROR));
        }
    }
}

//
// End of File
//

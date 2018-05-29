//#############################################################################
//
// FILE:   main.c
//
// TITLE:  SmartWinchDualCore
//
// MEng Final Year Project
//
// This project was prepared by importing the single-core driverlib empty project.
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.03.00.00 $
// $Release Date: Thu Dec  7 18:51:32 CST 2017 $
// $Copyright:
// Copyright (C) 2013-2017 Texas Instruments Incorporated - http://www.ti.com/
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
#include "includes.h"

//
// Globals
//
//temporary globals
char *msg; //for fixed const msg
char str[128]; //for runtime msg
uint16_t receivedChar[32]; //for general purpose reception


//todo: fetch value from FRAM
void init_settings()
{
    //legacy
    modbus_holding_regs[Kp] = 4096;
    modbus_holding_regs[Ki] = 0;
    modbus_holding_regs[Kd] = 30000;

    //legacy
    pid1.Umax = 16777215.0f;
    pid1.Umin = -16777215.0f;

    modbus_holding_regs[Field_Length] = 1000;
    modbus_holding_regs[Field_Width] = 1000;

    //default motion settings - 19th feb 2018
    modbus_holding_regs[Max_Velocity] = 50; /* mm/s */
    modbus_holding_regs[Max_Acceleration] = 50; /* mm/s^2 */
    modbus_holding_regs[scaled_velocity] = modbus_holding_regs[Max_Velocity]; /* mm/s */

    //30th march 2018
    modbus_holding_regs[Kp_velocity] = 16;
    modbus_holding_regs[Ki_velocity] = 0;
    modbus_holding_regs[Kd_velocity] = 0;
    modbus_holding_regs[Max_Encoder_Feedrate] = 250; /* no load max speed in mm/s */

    modbus_holding_regs[Kp_position] = 2048;
    modbus_holding_regs[Ki_position] = 0;
    modbus_holding_regs[Kd_position] = 0;   

    modbus_holding_regs[load_cell_cal] = 31000; 

    modbus_holding_regs[Encoder_Radius] = 7500;
    modbus_holding_regs[Encoder_Pulse] = 3840;


    modbus_holding_regs[minimum_duty_cycle] = 5; // 5% pwm duty cycle
    modbus_holding_regs[minimum_tension] = 100; //100 grams
    modbus_holding_regs[maximum_tension] = 1000; //1kg

    modbus_holding_regs[Current_Waypoints_Pointer] = 0;
    modbus_holding_regs[Dwell_Time] = 1;
    
    //Defines motor coordinates
    //5 input lengths required in metres as follows:
    //Motor A to Motor B
    //Motor C to Motor D
    //Motor A to Motor D
    //Motor A to Motor C
    //Motor B to Motor D
    //This fully charactises the layout and coordinates are set to allow for...
    //any quadrilateral motor shape
    set_motor_coord(len_ab,len_cd,len_ad,len_ac,len_bd);
}

void blinking_led()
{
    static uint32_t loop_counter = 0;
    if(loop_counter == 0)
    {
        GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
        GPIO_togglePin(DEVICE_GPIO_PIN_LED2);
    }        
    loop_counter+=1;

    if (!dip_switch.BIT6)   
        loop_counter&=0x0f;
    else 
        loop_counter&=0xff;
}

//
// Main
//
void main(void)
{    
    //default calibration values
    init_settings();

    //init required modules
    init_smartwinch();

    // Send starting message. todo: send help messages instead
    msg = "https://github.com/AfdhalAtiffTan/FYP_SmartWinch\n\r"; //50 characters
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 50);    

    modbus_holding_regs[Current_Force_Winch0] = 10608;
    modbus_holding_regs[Current_Force_Winch1] = 5258;
    modbus_holding_regs[Current_Force_Winch2] = 1990;
    modbus_holding_regs[Current_Force_Winch3] = 10608;
    modbus_holding_regs[Current_Length_Winch0] = 7870;
    modbus_holding_regs[Current_Length_Winch1] = 18493;
    modbus_holding_regs[Current_Length_Winch2] = 21499;
    modbus_holding_regs[Current_Length_Winch3] = 13.490;
    modbus_holding_regs[Field_Length] = 1000;


    //main loop
    while(1)
    {
        //read_DIP_switch();

        blinking_led();

        //address switch status
        modbus_holding_regs[Winch_ID] = dip_switch.ADDRESS_SWITCH;

        //service modbus request if available
        modbusRTU_Update((modbus_holding_regs[Winch_ID]+1), modbus_holding_regs, MB_HREGS);
        //modbusRTU_Update(1, modbus_holding_regs, MB_HREGS);

        //force system reset
        if(modbus_holding_regs[Soft_Reset] != 0 || (GPIO_readPin(DEVICE_GPIO_PIN_EMERGENCY_STOP) == 0))
        {
            modbus_holding_regs[Soft_Reset] = 0;
            SysCtl_resetDevice();
        }
        
        //debugging mode, if enabled, the modbus will be on uart_usb instead of on uart_wireless
        DEBUGGING = dip_switch.DEBUG_SWITCH;

        task_scheduler_handler(); // main program    

        if(dip_switch.BIT7) //used to test matt's maths
        {
            GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1); //tic
            //Tensions x4, Sagged Lengths x4, Distance
            XYZ_coord_struct coord_out;        
            coord_out = tenandsag2coord((float)modbus_holding_regs[Current_Force_Winch0]*0.0098066500286389f,
                                        (float)modbus_holding_regs[Current_Force_Winch1]*0.0098066500286389f,
                                        (float)modbus_holding_regs[Current_Force_Winch2]*0.0098066500286389f,
                                        (float)modbus_holding_regs[Current_Force_Winch3]*0.0098066500286389f,
                                        (float)modbus_holding_regs[Current_Length_Winch0]/1000.0f,
                                        (float)modbus_holding_regs[Current_Length_Winch1]/1000.0f,
                                        (float)modbus_holding_regs[Current_Length_Winch2]/1000.0f,
                                        (float)modbus_holding_regs[Current_Length_Winch3]/1000.0f,
                                        (float)modbus_holding_regs[Field_Length]/1000.0f);

            coord_out.X *= 1000;
            coord_out.Y *= 1000;
            coord_out.Z *= 1000;        
            modbus_holding_regs[kinematics_test_X] = (int) coord_out.X;
            modbus_holding_regs[kinematics_test_Y] = (int) coord_out.Y;
            modbus_holding_regs[kinematics_test_Z] = (int) coord_out.Z;    
            modbus_holding_regs[kinematics_test_U] = (int) (coord_out.uplift/0.0098066500286389f);                                  
            GPIO_writePin(DEVICE_GPIO_PIN_LED1, 0); //toc


            GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1); //tic
            //Takes new coordintes XYZ, distance and uplift from previous function
            length4_struct length_out;
            length_out = coord2ten_sag( (float)modbus_holding_regs[Target_X]/1000.0f,
                                        (float)modbus_holding_regs[Target_Y]/1000.0f,
                                        (float)modbus_holding_regs[Target_Z]/1000.0f,
                                        (float)modbus_holding_regs[Field_Length]/1000.0f,
                                        coord_out.uplift);

            length_out.lengtha *= 1000;
            length_out.lengthb *= 1000;
            length_out.lengthc *= 1000;
            length_out.lengthd *= 1000;

            modbus_holding_regs[kinematics_test_A] = (int) length_out.lengtha;
            modbus_holding_regs[kinematics_test_B] = (int) length_out.lengthb;
            modbus_holding_regs[kinematics_test_C] = (int) length_out.lengthc;
            modbus_holding_regs[kinematics_test_D] = (int) length_out.lengthd;

            GPIO_writePin(DEVICE_GPIO_PIN_LED2, 0); //toc
        }

        static uint32_t prev_tick1 = 0;
        if(systick() - prev_tick1 > 5000)
        {
            prev_tick1 = systick();
            FM25W256_write(0xAA, modbus_holding_regs[fram_test_w]);
            modbus_holding_regs[fram_test_r] = FM25W256_read(0xAA);
        }        
    }
}

//
// End of File
//

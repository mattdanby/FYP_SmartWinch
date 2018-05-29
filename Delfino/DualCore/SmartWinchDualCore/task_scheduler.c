/*
 * task_scheduler.c
 *
 *  Created on: 17 Mar 2018
 *      Author: AfdhalAtiffTan
 */

#include "task_scheduler.h"

DIP_switch_t dip_switch; //global


char homing_workaround = 1; //as a workaround to mcp266 glitch

void homing_routine() //with mcp266 apis
{
    static bool coordinate_changed = 0;
    static length4_struct homed_cable_lengths;

    //homing via coordinate
    if(modbusRTU_written_register_flags[Current_X] || modbusRTU_written_register_flags[Current_Y] || modbusRTU_written_register_flags[Current_Z])
    {
        //clear flags
        modbusRTU_written_register_flags[Current_X] = 0;
        modbusRTU_written_register_flags[Current_Y] = 0;
        modbusRTU_written_register_flags[Current_Z] = 0;

        coordinate_changed = 1; //set flag

        modbus_holding_regs[Target_X] = modbus_holding_regs[Current_X];
        modbus_holding_regs[Target_Y] = modbus_holding_regs[Current_Y];
        modbus_holding_regs[Target_Z] = modbus_holding_regs[Current_Z];
        
        

        homed_cable_lengths = XYZ_to_length4(  	(float) modbus_holding_regs[Current_X],
                                                (float) modbus_holding_regs[Current_Y], 
                                                (float) modbus_holding_regs[Current_Z]);

        modbus_holding_regs[Target_Length_Winch0] = (int) homed_cable_lengths.lengtha;
        modbus_holding_regs[Target_Length_Winch1] = (int) homed_cable_lengths.lengthb;
        modbus_holding_regs[Target_Length_Winch2] = (int) homed_cable_lengths.lengthc;
        modbus_holding_regs[Target_Length_Winch3] = (int) homed_cable_lengths.lengthd;

        modbus_holding_regs[Current_Length_Winch0] = modbus_holding_regs[Target_Length_Winch0];
        modbus_holding_regs[Current_Length_Winch1] = modbus_holding_regs[Target_Length_Winch1];
        modbus_holding_regs[Current_Length_Winch2] = modbus_holding_regs[Target_Length_Winch2];
        modbus_holding_regs[Current_Length_Winch3] = modbus_holding_regs[Target_Length_Winch3];

        switch(modbus_holding_regs[Winch_ID])
        {
            case 0:
            {
                modbus_holding_regs[Current_Encoder_Count] = modbus_holding_regs[Current_Length_Winch0];
                break;
            }
            case 1:
            {
                modbus_holding_regs[Current_Encoder_Count] = modbus_holding_regs[Current_Length_Winch1];
                break;
            }
            case 2:
            {
                modbus_holding_regs[Current_Encoder_Count] = modbus_holding_regs[Current_Length_Winch2];
                break;
            }
            case 3:
            {
                modbus_holding_regs[Current_Encoder_Count] = modbus_holding_regs[Current_Length_Winch3];
                break;
            }
        }
    }

    //homing via zero switch
    if(!GPIO_readPin(DEVICE_GPIO_PIN_ZERO_TETHER))
    {
        coordinate_changed = 1;
        modbus_holding_regs[Current_Encoder_Count] = 0;
    }

    //homing via length
    if(modbusRTU_written_register_flags[Current_Encoder_Count] || coordinate_changed)
    {
        coordinate_changed = 0; //clear flag        
        modbusRTU_written_register_flags[Current_Encoder_Count] = 0; //clear flag

        if (modbus_holding_regs[Current_Encoder_Count] == 0)
            modbus_holding_regs[Current_Encoder_Count] = 1; //0mm doesnt work on mcp266

        modbus_holding_regs[Target_Setpoint] = modbus_holding_regs[Current_Encoder_Count];        
        //MotionProfile_reset_position((float) modbus_holding_regs[Target_Setpoint]); //used to remove jerk, not sure if it work yet

        int32_t homed_length = length_to_encoder_pulses(modbus_holding_regs[Current_Encoder_Count]);

        //reset the encoder count using the requested length
        RoboClaw_SetEncM1(RoboClaw_Address, homed_length);

        homing_workaround = 0;
    }
}

void check_current_coordinate()
{
    if (modbusRTU_written_register_flags[Current_Length_Winch0] ||
        modbusRTU_written_register_flags[Current_Length_Winch1] ||
        modbusRTU_written_register_flags[Current_Length_Winch2] ||
        modbusRTU_written_register_flags[Current_Length_Winch3] ) //if current length was updated
    {
        volatile static XYZ_coord_struct actual_point;

        //clear flags
        modbusRTU_written_register_flags[Current_Length_Winch0] = 0;
        modbusRTU_written_register_flags[Current_Length_Winch1] = 0;
        modbusRTU_written_register_flags[Current_Length_Winch2] = 0;
        modbusRTU_written_register_flags[Current_Length_Winch3] = 0;

        //calculate current point based on cable length
        //future: use tension compensated estimation
        actual_point = tenandsag2coord( (float) modbus_holding_regs[Current_Force_Winch0]*0.0098066500286389f,
                                        (float) modbus_holding_regs[Current_Force_Winch1]*0.0098066500286389f,
                                        (float) modbus_holding_regs[Current_Force_Winch2]*0.0098066500286389f,
                                        (float) modbus_holding_regs[Current_Force_Winch3]*0.0098066500286389f,
                                        (float) modbus_holding_regs[Current_Length_Winch0]/1000.0f,
                                        (float) modbus_holding_regs[Current_Length_Winch1]/1000.0f,
                                        (float) modbus_holding_regs[Current_Length_Winch2]/1000.0f,
                                        (float) modbus_holding_regs[Current_Length_Winch3]/1000.0f);

        modbus_holding_regs[Current_X] =  (signed int) actual_point.X;
        modbus_holding_regs[Current_Y] =  (signed int) actual_point.Y;
        modbus_holding_regs[Current_Z] =  (signed int) actual_point.Z;    
    }    
}

//used to read all adc on J21 (4 channels) round-robin
void read_all_adc() //legacy
{
    //if the conversion was done, fetch all adc results
    if(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1))
    {
        modbus_holding_regs[ADC0] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
        modbus_holding_regs[ADC1] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
        modbus_holding_regs[ADC2] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);
        modbus_holding_regs[ADC3] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER3);
        ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
    }    
}

void relative_control()
{
    //relative control
    //to do: add simple boundary check to avoid overflow

    // manual tether control via switches
    if(modbus_holding_regs[tether_reached_target])
    {
        if(GPIO_readPin(DEVICE_GPIO_PIN_REEL_OUT) == 0)
            modbus_holding_regs[Target_Setpoint_Offset] += 10;
        if(GPIO_readPin(DEVICE_GPIO_PIN_REEL_IN) == 0)
            modbus_holding_regs[Target_Setpoint_Offset] -= 10;
    }

    modbus_holding_regs[Target_Setpoint] += modbus_holding_regs[Target_Setpoint_Offset];
    modbus_holding_regs[Target_X] += modbus_holding_regs[Target_X_Offset];
    modbus_holding_regs[Target_Y] += modbus_holding_regs[Target_Y_Offset];
    modbus_holding_regs[Target_Z] += modbus_holding_regs[Target_Z_Offset];

    modbus_holding_regs[Target_Setpoint_Offset] = 0;
    modbus_holding_regs[Target_X_Offset] = 0;
    modbus_holding_regs[Target_Y_Offset] = 0;
    modbus_holding_regs[Target_Z_Offset] = 0;
}

void manual_control()
{
    static length4_struct target_cable_lengths;
    static XYZ_coord_struct target_point;
    static XYZ_coord_struct cur_target_point;

    relative_control();

    if(modbusRTU_Written && (modbusRTU_written_register_flags[Target_Setpoint]) || modbusRTU_written_register_flags[Target_Setpoint_Offset]) //if length changed
    {
        //clear flags
        modbusRTU_Written = 0;
        modbusRTU_written_register_flags[Target_Setpoint] = 0;
        modbusRTU_written_register_flags[Target_Setpoint_Offset] = 0;

        //update target waypoint
        modbus_holding_regs[Target_Length_Winch0 + modbus_holding_regs[Winch_ID]] = modbus_holding_regs[Target_Setpoint]; 

        //to do: use tension compensated function        
        target_point = length4_to_XYZ(  (float) modbus_holding_regs[Target_Length_Winch0]/1000.0f,
                                        (float) modbus_holding_regs[Target_Length_Winch0]/1000.0f,
                                        (float) modbus_holding_regs[Target_Length_Winch0]/1000.0f,
                                        (float) modbus_holding_regs[Target_Length_Winch0]/1000.0f);

        modbus_holding_regs[Target_X] =  (signed int) target_point.X;
        modbus_holding_regs[Target_Y] =  (signed int) target_point.Y;
        modbus_holding_regs[Target_Z] =  (signed int) target_point.Z;

        modbus_holding_regs[scaled_velocity] = modbus_holding_regs[Max_Velocity];
    }

    if(modbusRTU_Written && (   modbusRTU_written_register_flags[Target_X]          ||
                                modbusRTU_written_register_flags[Target_Y]          ||
                                modbusRTU_written_register_flags[Target_Z]          ||
                                modbusRTU_written_register_flags[Target_X_Offset]   ||
                                modbusRTU_written_register_flags[Target_Y_Offset]   ||
                                modbusRTU_written_register_flags[Target_Z_Offset]   )) //if waypoint changed
    {
        //clear flags
        modbusRTU_Written = 0;
        modbusRTU_written_register_flags[Target_X] = 0;
        modbusRTU_written_register_flags[Target_Y] = 0;
        modbusRTU_written_register_flags[Target_Z] = 0;
        modbusRTU_written_register_flags[Target_X_Offset] = 0;
        modbusRTU_written_register_flags[Target_Y_Offset] = 0;
        modbusRTU_written_register_flags[Target_Z_Offset] = 0;

        //update target lengths
        //Finds current coordinates and from that, relative uplift
        cur_target_point = tenandsag2coord( (float) modbus_holding_regs[Current_Force_Winch0]*0.0098066500286389f,
                                            (float) modbus_holding_regs[Current_Force_Winch1]*0.0098066500286389f,
                                            (float) modbus_holding_regs[Current_Force_Winch2]*0.0098066500286389f,
                                            (float) modbus_holding_regs[Current_Force_Winch3]*0.0098066500286389f,
                                            (float) modbus_holding_regs[Current_Length_Winch0]/1000.0f,
                                            (float) modbus_holding_regs[Current_Length_Winch1]/1000.0f,
                                            (float) modbus_holding_regs[Current_Length_Winch2]/1000.0f,
                                            (float) modbus_holding_regs[Current_Length_Winch3]/1000.0f);
        
        //Uses calculated uplift to find tether lengths for target coordinate
        target_cable_lengths = coord2ten_sag((float) modbus_holding_regs[Target_X],
                                             (float) modbus_holding_regs[Target_Y],
                                             (float) modbus_holding_regs[Target_Z],
                                             (float) cur_target_point.uplift);


        modbus_holding_regs[Target_Length_Winch0] = target_cable_lengths.lengtha;
        modbus_holding_regs[Target_Length_Winch1] = target_cable_lengths.lengthb;
        modbus_holding_regs[Target_Length_Winch2] = target_cable_lengths.lengthc;
        modbus_holding_regs[Target_Length_Winch3] = target_cable_lengths.lengthd;

        update_scaled_velocity( target_cable_lengths.lengtha, 
                                target_cable_lengths.lengthb, 
                                target_cable_lengths.lengthc, 
                                target_cable_lengths.lengthd);

        switch(modbus_holding_regs[Winch_ID])
        {
            case 0:
            {
                modbus_holding_regs[Target_Setpoint] = target_cable_lengths.lengtha;
                break;
            }
            case 1:
            {
                modbus_holding_regs[Target_Setpoint] = target_cable_lengths.lengthb;
                break;
            }
            case 2:
            {
                modbus_holding_regs[Target_Setpoint] = target_cable_lengths.lengthc;
                break;
            }
            case 3:
            {
                modbus_holding_regs[Target_Setpoint] = target_cable_lengths.lengthd;
                break;
            }
        }                                            
    }  


    if(modbusRTU_Written && (   modbusRTU_written_register_flags[Target_Length_Winch0] ||
                                modbusRTU_written_register_flags[Target_Length_Winch1] ||
                                modbusRTU_written_register_flags[Target_Length_Winch2] ||
                                modbusRTU_written_register_flags[Target_Length_Winch3] )) //if Target_Length_Winchx changed, transfer it to Target_Setpoint appropriately
    {
        switch(modbus_holding_regs[Winch_ID])
        {
            case 0:
            {
                if(modbusRTU_written_register_flags[Target_Length_Winch0])
                    modbus_holding_regs[Target_Setpoint] = modbus_holding_regs[Target_Length_Winch0];
                break;
            }
            case 1:
            {
                if(modbusRTU_written_register_flags[Target_Length_Winch1])
                    modbus_holding_regs[Target_Setpoint] = modbus_holding_regs[Target_Length_Winch1];
                break;
            }
            case 2:
            {
                if(modbusRTU_written_register_flags[Target_Length_Winch2])
                    modbus_holding_regs[Target_Setpoint] = modbus_holding_regs[Target_Length_Winch2];
                break;
            }
            case 3:
            {
                if(modbusRTU_written_register_flags[Target_Length_Winch3])
                    modbus_holding_regs[Target_Setpoint] = modbus_holding_regs[Target_Length_Winch3];
                break;
            }
        }

        //update target coordinate based on the latest desired lengths
        target_point = length4_to_XYZ(  (float) modbus_holding_regs[Target_Length_Winch0],
                                        (float) modbus_holding_regs[Target_Length_Winch1],
                                        (float) modbus_holding_regs[Target_Length_Winch2],
                                        (float) modbus_holding_regs[Target_Length_Winch3]);

        modbus_holding_regs[Target_X] =  (signed int) target_point.X;
        modbus_holding_regs[Target_Y] =  (signed int) target_point.Y;
        modbus_holding_regs[Target_Z] =  (signed int) target_point.Z;     

        //clear flags
        modbusRTU_Written = 0;
        modbusRTU_written_register_flags[Target_Length_Winch0] = 0;
        modbusRTU_written_register_flags[Target_Length_Winch1] = 0;
        modbusRTU_written_register_flags[Target_Length_Winch2] = 0;
        modbusRTU_written_register_flags[Target_Length_Winch3] = 0; 

        modbus_holding_regs[scaled_velocity] = modbus_holding_regs[Max_Velocity];       
    }
}

void auto_mode() //autonomous mode
{
    waypoint_follower();
}


void read_current_cable_length_from_mcp266()
{
    uint16_t status;
    bool valid;
    int32_t count = RoboClaw_ReadEncM1(RoboClaw_Address, &status, &valid);

    if(valid)
        modbus_holding_regs[Current_Encoder_Count] = encoder_pulses_to_length(count);
    
    //update register accordingly
    modbus_holding_regs[Current_Length_Winch0 + modbus_holding_regs[Winch_ID]] = modbus_holding_regs[Current_Encoder_Count];
}

void update_mcp266_pids()
{
    //update velocity pid
    if( modbusRTU_written_register_flags[Kp_velocity]        ||
        modbusRTU_written_register_flags[Ki_velocity]        ||
        modbusRTU_written_register_flags[Kd_velocity]        ||
        modbusRTU_written_register_flags[Max_Encoder_Feedrate])
    {
        //clear flags
        modbusRTU_written_register_flags[Kp_velocity] = 0;
        modbusRTU_written_register_flags[Ki_velocity] = 0;
        modbusRTU_written_register_flags[Kd_velocity] = 0;
        modbusRTU_written_register_flags[Max_Encoder_Feedrate] = 0;

        RoboClaw_SetM1VelocityPID(RoboClaw_Address,
                                  (float)modbus_holding_regs[Kp_velocity],
                                  (float)modbus_holding_regs[Ki_velocity],
                                  (float)modbus_holding_regs[Kd_velocity],
                                  length_to_encoder_pulses(modbus_holding_regs[Max_Encoder_Feedrate]));
    }

    //update position pid
    if( modbusRTU_written_register_flags[Kp_position]   ||
        modbusRTU_written_register_flags[Ki_position]   ||
        modbusRTU_written_register_flags[Kd_position]   )
    {
        //clear flags
        modbusRTU_written_register_flags[Kp_position] = 0;
        modbusRTU_written_register_flags[Ki_position] = 0;
        modbusRTU_written_register_flags[Kd_position] = 0;

        RoboClaw_SetM1PositionPID(RoboClaw_Address,
                                  (float)modbus_holding_regs[Kp_position],
                                  (float)modbus_holding_regs[Ki_position],
                                  (float)modbus_holding_regs[Kd_position],
                                  255,  //Integral max - not sure how to set this
                                  0,  //deadzone - not sure how to set this
                                  0x80000000,  //min limit
                                  0x7fffffff); //max limit                               
    }     
}

void read_mcp266_pids()
{
    float pid_temp[3]; uint32_t qpps_temp; bool valid;
    valid = RoboClaw_ReadM1VelocityPID( RoboClaw_Address,
                                        &pid_temp[0],
                                        &pid_temp[1],
                                        &pid_temp[2],
                                        &qpps_temp);
    if(valid)
    {
        modbus_holding_regs[Kp_velocity] = (int) pid_temp[0]; 
        modbus_holding_regs[Ki_velocity] = (int) pid_temp[1];
        modbus_holding_regs[Kd_velocity] = (int) pid_temp[2];
        modbus_holding_regs[Max_Encoder_Feedrate] = encoder_pulses_to_length(qpps_temp);
    }
    

    uint32_t misc_temp[4];
    valid = RoboClaw_ReadM1PositionPID( RoboClaw_Address,
                                        &pid_temp[0],
                                        &pid_temp[1],
                                        &pid_temp[2],
                                        &misc_temp[0], //KiMax
                                        &misc_temp[1], //DeadZone
                                        &misc_temp[2], //Min
                                        &misc_temp[3]); //Max
    if(valid)
    {                                    
        modbus_holding_regs[Kp_position] = (int) pid_temp[0]; 
        modbus_holding_regs[Ki_position] = (int) pid_temp[1];
        modbus_holding_regs[Kd_position] = (int) pid_temp[2];
    }
}

void read_mcp266_pwm()
{
    int16_t pwm[2];
    if(RoboClaw_ReadPWMs(RoboClaw_Address, &pwm[0], &pwm[1]))
        modbus_holding_regs[Current_PWM] = (signed int) (((float)pwm[0])/327.67f);
}

void read_load_cell()
{
    static uint32_t last_sucessful_time_stamp = 0;

    //if data is ready, read it
    if(!GPIO_readPin(DEVICE_GPIO_PIN_SPIIRQA))
    {
        SPI_resetRxFIFO (SPIA_BASE); //clear rx buffer for new data 
        SPI_writeDataBlockingFIFO (SPIA_BASE, 0x0000); //read msb
        SPI_writeDataBlockingFIFO (SPIA_BASE, 0x0000); //read middle byte
        SPI_writeDataBlockingFIFO (SPIA_BASE, 0x0000); //read lsb
        SPI_writeDataBlockingFIFO (SPIA_BASE, 0x0000); //dummy
        while(SPI_isBusy (SPIA_BASE)); //stay until all data is transmited

        //irq pin should be high after the data has been read
        //if not, maybe there is an error
        //check for always-low pin
        if(!GPIO_readPin(DEVICE_GPIO_PIN_SPIIRQA))
            modbus_holding_regs[load_cell_error] = 1;
        else
        {
            last_sucessful_time_stamp = systick();
            modbus_holding_regs[load_cell_error] = 0;
        }
            
        static uint32_t rcv_spi[4];
        uint16_t i=0; //received byte count
        while (SPI_getRxFIFOStatus(SPIA_BASE)) //read all received byte and put it in a buffer
        {                
            rcv_spi[i++] = SPI_readDataBlockingFIFO(SPIA_BASE);                                
        }

        //ads1220 gives signed 24-bit data, put it left-justified
        int32_t signed_load_cell = ((rcv_spi[0]<<24) ^ (rcv_spi[1]<<16) ^ (rcv_spi[2]<<8));
        signed_load_cell>>=8; //this should convert signed 24bit into signed 32bit (2's complement)

        static int32_t offset_load_cell = 0; 

        if(modbus_holding_regs[load_cell_zero]==LOAD_CELL_ZEROING_REQUEST)
        {
            modbus_holding_regs[load_cell_zero] = LOAD_CELL_ZEROED;
            offset_load_cell = signed_load_cell;
            //todo: put offset in fram
            modbus_holding_regs[load_cell_H] = (offset_load_cell>>16);
            modbus_holding_regs[load_cell_L] = (offset_load_cell & 0x0000ffff);
        }

        int32_t tare_load_cell = signed_load_cell - offset_load_cell;

        int32_t calibrated_load_cell = tare_load_cell / ((((unsigned int)modbus_holding_regs[load_cell_cal]) + 1) / 1000);

        if(calibrated_load_cell<0)
        {
            modbus_holding_regs[load_cell_neg] = 1; //negative force flag
            calibrated_load_cell*=-1;
        }
        else
            modbus_holding_regs[load_cell_neg] = 0;

        //full 24bit reading
        modbus_holding_regs[load_cell_H] = (signed int)((calibrated_load_cell>>16)&0xffff);
        modbus_holding_regs[load_cell_L] = (signed int)(calibrated_load_cell&0xffff); 

        //only keep the first 65kg
        modbus_holding_regs[Current_Force_Winch0+modbus_holding_regs[Winch_ID]] = modbus_holding_regs[load_cell_L]; 

        //underload check
        if (modbus_holding_regs[load_cell_L] < modbus_holding_regs[minimum_tension])
            modbus_holding_regs[underload_error] = 1;
        else
            modbus_holding_regs[underload_error] = 0;

        //overload check
        if (modbus_holding_regs[load_cell_L] > modbus_holding_regs[maximum_tension])    
            modbus_holding_regs[overload_error] = 1;
        else
            modbus_holding_regs[overload_error] = 0;
    }   
    //checking for always-high pin
    else
    {
        //irq pin should go low periodically
        if (systick() - last_sucessful_time_stamp > 20000) //if no data for more than 4 secs
            modbus_holding_regs[load_cell_error] = 1;
    }
}

void is_tether_close_to_target_length()
{
    //set flag if length error is less than 2cm
    if(abs(modbus_holding_regs[Target_Setpoint] - modbus_holding_regs[Current_Encoder_Count]) < 20)
        modbus_holding_regs[tether_reached_target] = 1;
    else
        modbus_holding_regs[tether_reached_target] = 0;
}


//must read loadcell first before calling this function
void zero_force_mode()
{
    //if load cell is zeroed and the zero_force switch is activated
    if(modbus_holding_regs[load_cell_zero] == LOAD_CELL_ZEROED && !GPIO_readPin(DEVICE_GPIO_PIN_ZERO_FORCE))
    {
        if(modbus_holding_regs[load_cell_L] > 100) //if pulling force is higher than 100grams
        {
            if(modbus_holding_regs[tether_reached_target]) //only add offset when the length is near Target_Setpoint
            {
                modbus_holding_regs[Target_Setpoint_Offset] += 10; //add 1cm when pulled
            }
        }
    }
}

void read_DIP_switch()
{
    //active-low switches
    dip_switch.BIT0 = ~GPIO_readPin(DEVICE_GPIO_PIN_B0);
    dip_switch.BIT1 = ~GPIO_readPin(DEVICE_GPIO_PIN_B1);
    dip_switch.BIT2 = ~GPIO_readPin(DEVICE_GPIO_PIN_B2);
    dip_switch.BIT3 = ~GPIO_readPin(DEVICE_GPIO_PIN_B3);
    dip_switch.BIT4 = ~GPIO_readPin(DEVICE_GPIO_PIN_B4);
    dip_switch.BIT5 = ~GPIO_readPin(DEVICE_GPIO_PIN_B5);
    dip_switch.BIT6 = ~GPIO_readPin(DEVICE_GPIO_PIN_B6);
    dip_switch.BIT7 = ~GPIO_readPin(DEVICE_GPIO_PIN_B7); 
    modbus_holding_regs[DIP_Switch_Status] = dip_switch.switch_byte;   
}

void underrun_error_check()
{    
    static uint32_t previous_systick = 0;
    if(systick() - previous_systick > 2500) //check underrun every 500ms
    {
        previous_systick = systick();

        static int previous_measured_cable_length = 0;
        if (abs(modbus_holding_regs[Current_PWM]) > modbus_holding_regs[minimum_duty_cycle]) //if the motor is spinning
        {
            if (modbus_holding_regs[Current_Encoder_Count] == previous_measured_cable_length) //if the cable doesnt move
                modbus_holding_regs[underrun_error] = 1; //set flag
            else
                modbus_holding_regs[underrun_error] = 0; //clear flag
            
            previous_measured_cable_length = modbus_holding_regs[Current_Encoder_Count];
        }
        else //check for motor is not powered
        {
            if (modbus_holding_regs[Current_Encoder_Count] != previous_measured_cable_length) //if the cable moves
                modbus_holding_regs[underrun_error] = 0; //clear flag
       
            previous_measured_cable_length = modbus_holding_regs[Current_Encoder_Count];
        }
    }
}

void task_scheduler_handler()
{
    //future: use state machine to read sequentially
    read_DIP_switch();
    is_tether_close_to_target_length();
    homing_routine();
    read_current_cable_length_from_mcp266();
    check_current_coordinate();
    update_mcp266_pids();
    read_mcp266_pids();
    read_load_cell();
    zero_force_mode();

    if(modbus_holding_regs[interwinch_comms])
        interwinch_comms_handler();
    
    //simple_homing_routine();
    //fetch_current_rpm();
    //check_current_coordinate();
    //read_all_adc();

    if(modbus_holding_regs[Follow_Waypoints] == 0) //if in manual mode, i.e. not using waypoint buffer
    {
        manual_control();
    }    
    else // if autonomous mode activated, i.e. using waypoint buffer
    {
        waypoint_follower();
    }

    //mcp266 bug workaround
    //because it doesnt work when the position is set to 0
    if(modbus_holding_regs[Target_Setpoint] == 0)
    {
        homing_workaround = 0;
        modbus_holding_regs[Target_Setpoint] = 1;
    }        
                    
    motor_driving_with_safety_check();

    homing_workaround = 1;  //this workaround is for removing undesired motor jerk

    read_mcp266_pwm();  
    underrun_error_check();                                    
}


/*
 * autonomous_mode.c
 *
 *  Created on: 16 May 2018
 *      Author: AfdhalAtiffTan
 */

#include "autonomous_mode.h"


//this function will only update the target setpoint in the hreg
void waypoint_follower()
{
    static length4_struct target_cable_lengths;
    static int previous_mode = 0;
    
    if (modbus_holding_regs[Current_Waypoints_Pointer] < modbus_holding_regs[Number_of_Waypoints])
    {
        modbus_holding_regs[Target_X] = waypoints(modbus_holding_regs[Current_Waypoints_Pointer], 0);
        modbus_holding_regs[Target_Y] = waypoints(modbus_holding_regs[Current_Waypoints_Pointer], 1);
        modbus_holding_regs[Target_Z] = waypoints(modbus_holding_regs[Current_Waypoints_Pointer], 2);
        
        if(dip_switch.BIT7) //used to test matt's maths
        {
            //Pythagorean maths
        target_cable_lengths = XYZ_to_length4(  modbus_holding_regs[Current_X],
                +                               modbus_holding_regs[Current_Y], 
                +                               modbus_holding_regs[Current_Z]);
        } else {

        //update target lengths
+        //Finds current coordinates and from that, relative uplift
+        cur_target_point = tenandsag2coord( (float) modbus_holding_regs[Current_Force_Winch0]*0.0098066500286389f,
+                                            (float) modbus_holding_regs[Current_Force_Winch1]*0.0098066500286389f,
+                                            (float) modbus_holding_regs[Current_Force_Winch2]*0.0098066500286389f,
+                                            (float) modbus_holding_regs[Current_Force_Winch3]*0.0098066500286389f,
+                                            modbus_holding_regs[Current_Length_Winch0],
+                                            modbus_holding_regs[Current_Length_Winch1],
+                                            modbus_holding_regs[Current_Length_Winch2],
+                                            modbus_holding_regs[Current_Length_Winch3]);
+        
+        //Uses calculated uplift to find tether lengths for target coordinate
+        target_cable_lengths = coord2ten_sag(modbus_holding_regs[Target_X],
+                                             modbus_holding_regs[Target_Y],
+                                             modbus_holding_regs[Target_Z],
+                                             cur_target_point.uplift);
        }

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
        
        if(modbus_holding_regs[tether_reached_target])
        {
            static uint32_t last_timestamp = 0;
            
            if(modbus_holding_regs[Follow_Waypoints] != previous_mode) //if mode was changed
                last_timestamp = systick();                
            previous_mode = modbus_holding_regs[Follow_Waypoints];

            if (systick() - last_timestamp > (5000*(uint32_t)modbus_holding_regs[Dwell_Time]))
            {
                modbus_holding_regs[Current_Waypoints_Pointer]+=1;
                last_timestamp = systick();
            }
        }               
    }
    else
    {
        modbus_holding_regs[Follow_Waypoints] = 0;
        modbus_holding_regs[Current_Waypoints_Pointer] = 0;
        previous_mode = 0;
    }
}


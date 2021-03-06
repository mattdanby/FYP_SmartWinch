#include "RoboClaw.h"

#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg



//private:
void crc_clear();
void crc_update (uint8_t data);
uint16_t crc_get();
bool write_n(uint8_t byte,...);
bool read_n(uint8_t byte,uint8_t address,uint8_t cmd,...);
uint32_t Read4_1(uint8_t address,uint8_t cmd,uint8_t *status,bool *valid);
uint32_t Read4(uint8_t address,uint8_t cmd,bool *valid);
uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid);
uint8_t Read1(uint8_t address,uint8_t cmd,bool *valid);


//
// Constructor
//
RoboClaw_RoboClaw(HardwareSerial *serial, uint32_t tout)
{
	timeout = tout;
	hserial = serial;

}


void RoboClaw_begin(long speed)
{
	if(hserial){
		hserial->begin(speed);
	}
#ifdef __AVR__
	else{
		sserial->begin(speed);
	}
#endif
}

bool RoboClaw_listen()
{
#ifdef __AVR__
	if(sserial){
		return sserial->listen();
	}
#endif
	return false;
}

bool RoboClaw_isListening()
{
#ifdef __AVR__
	if(sserial)
		return sserial->isListening();
#endif
	return false;
}

bool RoboClaw_overflow()
{
#ifdef __AVR__
	if(sserial)
		return sserial->overflow();
#endif
	return false;
}

int RoboClaw_peek()
{
	if(hserial)
		return hserial->peek();
#ifdef __AVR__
	else
		return sserial->peek();
#endif
}

size_t RoboClaw_write(uint8_t byte)
{
	if(hserial)
		return hserial->write(byte);
#ifdef __AVR__
	else
		return sserial->write(byte);
#endif
}

int RoboClaw_read()
{
	if(hserial)
		return hserial->read();
#ifdef __AVR__
	else
		return sserial->read();
#endif
}

int RoboClaw_available()
{
	if(hserial)
		return hserial->available();
#ifdef __AVR__
	else
		return sserial->available();
#endif
}

void RoboClaw_flush()
{
	if(hserial)
		hserial->flush();
}

int RoboClaw_read(uint32_t timeout)
{
	if(hserial){
		uint32_t start = micros();
		// Empty buffer?
		while(!hserial->available()){
		   if((micros()-start)>=timeout)
		      return -1;
		}
		return hserial->read();
	}
#ifdef __AVR__
	else{
		if(sserial->isListening()){
			uint32_t start = micros();
			// Empty buffer?
			while(!sserial->available()){
			   if((micros()-start)>=timeout)
				  return -1;
			}
			return sserial->read();
		}
	}
#endif
}

void RoboClaw_clear()
{
	if(hserial){
		while(hserial->available())
			hserial->read();
	}
#ifdef __AVR__
	else{
		while(sserial->available())
			sserial->read();
	}
#endif
}

void crc_clear()
{
	crc = 0;
}

void crc_update (uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}

uint16_t crc_get()
{
	return crc;
}

bool write_n(uint8_t cnt, ... )
{
	uint8_t trys=MAXRETRY;
	do{
		crc_clear();
		//send data with crc
		va_list marker;
		va_start( marker, cnt );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint8_t data = va_arg(marker, int);
			crc_update(data);
			write(data);
		}
		va_end( marker );              /* Reset variable arguments.      */
		uint16_t crc = crc_get();
		write(crc>>8);
		write(crc);
		if(read(timeout)==0xFF)
			return true;
	}while(trys--);
	return false;
}

bool read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();
		
		data=0;
		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		//send data with crc
		va_list marker;
		va_start( marker, cmd );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value=(uint32_t)data<<24;
			}
			else{
				break;
			}
			
			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data<<16;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data<<8;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data;
			}
			else{
				break;
			}

			*ptr = value;
		}
		va_end( marker );              /* Reset variable arguments.      */

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					return crc_get()==ccrc;
				}
			}
		}
	}while(trys--);

	return false;
}

uint8_t Read1(uint8_t address,uint8_t cmd,bool *valid){
	uint8_t crc;

	if(valid)
		*valid = false;
	
	uint8_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);
	
		data = read(timeout);
		crc_update(data);
		value=data;

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
	
	return false;
}

uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid){
	uint8_t crc;

	if(valid)
		*valid = false;
	
	uint16_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);
	
		data = read(timeout);
		crc_update(data);
		value=(uint16_t)data<<8;
		
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint16_t)data;
		}
		
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
		
	return false;
}

uint32_t Read4(uint8_t address, uint8_t cmd, bool *valid){
	uint8_t crc;
	
	if(valid)
		*valid = false;
	
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}
		
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}
		
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
	
	return false;
}

uint32_t Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid){
	uint8_t crc;

	if(valid)
		*valid = false;
	
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}
	
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			if(status)
				*status = data;
		}
				
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}

bool RoboClaw_ForwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1FORWARD,speed);
}

bool RoboClaw_BackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1BACKWARD,speed);
}

bool RoboClaw_SetMinVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINMB,voltage);
}

bool RoboClaw_SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXMB,voltage);
}

bool RoboClaw_ForwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2FORWARD,speed);
}

bool RoboClaw_BackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2BACKWARD,speed);
}

bool RoboClaw_ForwardBackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M17BIT,speed);
}

bool RoboClaw_ForwardBackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M27BIT,speed);
}

bool RoboClaw_ForwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFORWARD,speed);
}

bool RoboClaw_BackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDBACKWARD,speed);
}

bool RoboClaw_TurnRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDRIGHT,speed);
}

bool RoboClaw_TurnLeftMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLEFT,speed);
}

bool RoboClaw_ForwardBackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFB,speed);
}

bool RoboClaw_LeftRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLR,speed);
}

uint32_t RoboClaw_ReadEncM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ENC,status,valid);
}

uint32_t RoboClaw_ReadEncM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ENC,status,valid);
}

uint32_t RoboClaw_ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1SPEED,status,valid);
}

uint32_t RoboClaw_ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2SPEED,status,valid);
}

bool RoboClaw_ResetEncoders(uint8_t address){
	return write_n(2,address,RESETENC);
}

bool RoboClaw_ReadVersion(uint8_t address,char *version){
	uint8_t data;
	uint8_t trys=MAXRETRY;
	do{
		flush();

		data = 0;
		
		crc_clear();
		write(address);
		crc_update(address);
		write(GETVERSION);
		crc_update(GETVERSION);
	
		uint8_t i;
		for(i=0;i<48;i++){
			if(data!=-1){
				data=read(timeout);
				version[i] = data;
				crc_update(version[i]);
				if(version[i]==0){
					uint16_t ccrc;
					data = read(timeout);
					if(data!=-1){
						ccrc = data << 8;
						data = read(timeout);
						if(data!=-1){
							ccrc |= data;
							return crc_get()==ccrc;
						}
					}
					break;
				}
			}
			else{
				break;
			}
		}
	}while(trys--);
	
	return false;
}

bool RoboClaw_SetEncM1(uint8_t address, int32_t val){
	return write_n(6,address,SETM1ENCCOUNT,SetDWORDval(val));
}

bool RoboClaw_SetEncM2(uint8_t address, int32_t val){
	return write_n(6,address,SETM2ENCCOUNT,SetDWORDval(val));
}

uint16_t RoboClaw_ReadMainBatteryVoltage(uint8_t address,bool *valid){
	return Read2(address,GETMBATT,valid);
}

uint16_t RoboClaw_ReadLogicBatteryVoltage(uint8_t address,bool *valid){
	return Read2(address,GETLBATT,valid);
}

bool RoboClaw_SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINLB,voltage);
}

bool RoboClaw_SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXLB,voltage);
}

bool RoboClaw_SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool RoboClaw_SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t RoboClaw_ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t RoboClaw_ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ISPEED,status,valid);
}

bool RoboClaw_DutyM1(uint8_t address, uint16_t duty){
	return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool RoboClaw_DutyM2(uint8_t address, uint16_t duty){
	return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool RoboClaw_DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2){
	return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

bool RoboClaw_SpeedM1(uint8_t address, uint32_t speed){
	return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

bool RoboClaw_SpeedM2(uint8_t address, uint32_t speed){
	return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

bool RoboClaw_SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
	return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClaw_SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

bool RoboClaw_SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
bool RoboClaw_SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2){
	return write_n(14,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClaw_SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw_SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw_SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw_SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw_SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw_SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw_ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2){
	bool valid;
	uint16_t value = Read2(address,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}

bool RoboClaw_ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2){
	bool valid;
	uint32_t value = Read4(address,GETPWMS,&valid);
	if(valid){
		pwm1 = value>>16;
		pwm2 = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw_ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2){
	bool valid;
	uint32_t value = Read4(address,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>16;
		current2 = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw_SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

bool RoboClaw_SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw_DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel){
	return write_n(8,address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool RoboClaw_DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel){
	return write_n(8,address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool RoboClaw_DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){
	return write_n(14,address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2));
}

bool RoboClaw_ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw_ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw_SetMainVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClaw_SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClaw_ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(address,GETMINMAXMAINVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}
			
bool RoboClaw_ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(address,GETMINMAXLOGICVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw_SetM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){			
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw_SetM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){			
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw_ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboClaw_ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboClaw_SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClaw_SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClaw_SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool RoboClaw_SetM1DefaultAccel(uint8_t address, uint32_t accel){
	return write_n(6,address,SETM1DEFAULTACCEL,SetDWORDval(accel));
}

bool RoboClaw_SetM2DefaultAccel(uint8_t address, uint32_t accel){
	return write_n(6,address,SETM2DEFAULTACCEL,SetDWORDval(accel));
}

bool RoboClaw_SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){
	return write_n(5,address,SETPINFUNCTIONS,S3mode,S4mode,S5mode);
}

bool RoboClaw_GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode){
	uint8_t crc;
	bool valid = false;
	uint8_t val1,val2,val3;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(GETPINFUNCTIONS);
		crc_update(GETPINFUNCTIONS);
	
		data = read(timeout);
		crc_update(data);
		val1=data;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			val2=data;
		}
		
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			val3=data;
		}
		
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						S3mode = val1;
						S4mode = val2;
						S5mode = val3;
						return true;
					}
				}
			}
		}
	}while(trys--);
	
	return false;
}

bool RoboClaw_SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max){
	return write_n(4,address,SETDEADBAND,Min,Max);
}

bool RoboClaw_GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max){
	bool valid;
	uint16_t value = Read2(address,GETDEADBAND,&valid);
	if(valid){
		Min = value>>8;
		Max = value;
	}
	return valid;
}

bool RoboClaw_ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2){
	bool valid = read_n(2,address,GETENCODERS,&enc1,&enc2);
	return valid;
}

bool RoboClaw_ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2){
	bool valid = read_n(2,address,GETISPEEDS,&ispeed1,&ispeed2);
	return valid;
}

bool RoboClaw_RestoreDefaults(uint8_t address){
	return write_n(2,address,RESTOREDEFAULTS);
}

bool RoboClaw_ReadTemp(uint8_t address, uint16_t &temp){
	bool valid;
	temp = Read2(address,GETTEMP,&valid);
	return valid;
}

bool RoboClaw_ReadTemp2(uint8_t address, uint16_t &temp){
	bool valid;
	temp = Read2(address,GETTEMP2,&valid);
	return valid;
}

uint16_t RoboClaw_ReadError(uint8_t address,bool *valid){
	return Read2(address,GETERROR,valid);
}

bool RoboClaw_ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode){
	bool valid;
	uint16_t value = Read2(address,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

bool RoboClaw_SetM1EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool RoboClaw_SetM2EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM2ENCODERMODE,mode);
}

bool RoboClaw_WriteNVM(uint8_t address){
	return write_n(6,address,WRITENVM, SetDWORDval(0xE22EAB7A) );
}

bool RoboClaw_ReadNVM(uint8_t address){
	return write_n(2,address,READNVM);
}

bool RoboClaw_SetConfig(uint8_t address, uint16_t config){
	return write_n(4,address,SETCONFIG,SetWORDval(config));
}

bool RoboClaw_GetConfig(uint8_t address, uint16_t &config){
	bool valid;
	uint16_t value = Read2(address,GETCONFIG,&valid);
	if(valid){
		config = value;
	}
	return valid;
}

bool RoboClaw_SetM1MaxCurrent(uint8_t address,uint32_t max){
	return write_n(10,address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

bool RoboClaw_SetM2MaxCurrent(uint8_t address,uint32_t max){
	return write_n(10,address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

bool RoboClaw_ReadM1MaxCurrent(uint8_t address,uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,address,GETM1MAXCURRENT,&tmax,&dummy);
	if(valid)
		max = tmax;
	return valid;
}

bool RoboClaw_ReadM2MaxCurrent(uint8_t address,uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,address,GETM2MAXCURRENT,&tmax,&dummy);
	if(valid)
		max = tmax;
	return valid;
}

bool RoboClaw_SetPWMMode(uint8_t address, uint8_t mode){
	return write_n(3,address,SETPWMMODE,mode);
}

bool RoboClaw_GetPWMMode(uint8_t address, uint8_t &mode){
	bool valid;
	uint8_t value = Read1(address,GETPWMMODE,&valid);
	if(valid){
		mode = value;
	}
	return valid;
}

#ifndef UDGSHRIMPIII_H
#define UDGSHRIMPIII_H


//Shrimp error constants
#define S_ERROR_UNKNOWN_COMMAND 0x80 
#define S_ERROR_ARGUMENT 0x81
#define S_ERROR_I2C 0x82
#define S_ERROR_LIMIT_REACHED 0x83

#define RAW_BYTE unsigned char
//#define BYTE unsigned char
#define SIGNED_BYTE char
#define S_BUFFER_SIZE 1024
#define UNSIGNED_INTEGER32 unsigned int
#define UNSIGNED_INTEGER16 unsigned short

struct FirmwareVersion
{
	RAW_BYTE major;
	RAW_BYTE minor;
	RAW_BYTE patch;
};

/*Value F: current value of the front wheel encoder
Value FL: current value of the front left wheel encoder
Value FR: current value of the front right wheel encoder
Value BL: current value of the rear left wheel encoder
Value BR: current value of the rear right wheel encoder
Value B: current value of the rear wheel encoder
*/
struct WheelEncoders{
	UNSIGNED_INTEGER32 F;
	UNSIGNED_INTEGER32 FL;
	UNSIGNED_INTEGER32 FR;
	UNSIGNED_INTEGER32 BL;
	UNSIGNED_INTEGER32 BR;
	UNSIGNED_INTEGER32 B;
};

/*
Bit 0	ROB_ON	Robot is switched on
Bit 1	ROB_STOPPED	Robot is stopped by an emergency stop
Bit 2	IR_ENABLED	Infrared remote control is enabled
Bit 3 - 7	Unused */
struct RobotStatus
{
	unsigned char ROB_ON :1;
	unsigned char ROB_STOPPED :1;
	unsigned char IR_ENABLED :1;
	unsigned char UNUSED;
};

/*
Bit 0	ALL_OK	Power supply is ok, i.e. none of the bits below are set
Bit 1	IN_LOW	Battery voltage is below first security threshold Vlow
Bit 2	VIN_MIN	Battery voltage is below second security threshold Vmin, power to motors and servos will be switched off
Bit 3	VIN_SECURE	Battery voltage is below third security threshold Vsecure, power supply will shut down
Bit 4	VIN_ERROR	Battery voltage is low and 2nd derivative is too high, power supply will shut down
Bit 5	VIN_HI	Battery voltage is above Vmax, power supply will shut down
Bit 6	Unused	
Bit 7	D2_OVER	2nd derivative of battery voltage has been too high for a long time, power supply will shut down
*/
struct PowerSupplyStatus
{
	RAW_BYTE ALL_OK :1;
	RAW_BYTE IN_LOW :1;
	RAW_BYTE VIN_MIN :1;
	RAW_BYTE VIN_SECURE:1;
	RAW_BYTE VIN_ERROR :1;
	RAW_BYTE VIN_HI :1;
	RAW_BYTE unused :1;
	RAW_BYTE D2_OVER;
};

//Servo F (16) Servo B (16) Motor F (32) Motor L (32) Motor R (32) Motor B (32) 
/*
servoF: command to the front servo
servoB: command to the rear servo
motorF: command to the front motor
motorL: command to the left motors
motorR: command to the right motors
motorB: command to the rear motor

*/
struct SrvoAndMotorCommands 
{
	UNSIGNED_INTEGER16 servoF;
	UNSIGNED_INTEGER16 servoB;
	UNSIGNED_INTEGER32 motorF;
	UNSIGNED_INTEGER32 motorL;
	UNSIGNED_INTEGER32 motorR;
	UNSIGNED_INTEGER32 motorB;
};
/*
Bit 0	Reserved
Bit 1	nESTOP		State of active-low emergency stop input
Bit 2	GPIO		State of general-purpose input
Bit 3 - 7	Reserved
   */
struct DigitalInputs
{
	RAW_BYTE reserved0 :1;
	RAW_BYTE nESTOP :1;
	RAW_BYTE GPIO :1;
	RAW_BYTE reserved3;
};

struct RC5Frame
{
	RAW_BYTE address;
	RAW_BYTE data;
};
class UDG_ShrimpIII
{
public:
	FirmwareVersion firmwareVersion;
	SIGNED_BYTE velocity;
	SIGNED_BYTE steeringAngle;
	WheelEncoders wheelEncoders;
	RobotStatus robotStatus;
	PowerSupplyStatus powerSupplyStatus;
	float voltage; //battery voltage in volts
	SrvoAndMotorCommands servoAndMotorCommands;
	DigitalInputs digitalInputs;

	

	RAW_BYTE shrimpBuffer[S_BUFFER_SIZE];
	UDG_ShrimpIII(const char* serialPortName);
	~UDG_ShrimpIII();
	bool NoOperation();
	bool ReadFirmwareVersion();
	bool TurnRobotOn();
	bool TurnRobotOff();
	bool SetVelocityAndSteeringAngle(SIGNED_BYTE velocity, SIGNED_BYTE angle);
	bool ReadVelocityAndSteeringAngle();
	bool EmergencyStop();
	bool ReadWheelEncoderValues(WheelEncoders* encoders);
	bool ReadRobotStatus(RobotStatus* status);
	float ReadBatteryVoltage();
	bool ReadPowerSupplyStatus(PowerSupplyStatus* status);
	bool DisableInfraredRemoteControl();
	bool EnableInfraredRemoteControl();
	bool MutePowerSupplyBuzzer();
	bool UnmutePowerSupplyBuzzer();
	bool WriteAn8BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, RAW_BYTE value);
	bool ReadAn8BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, RAW_BYTE* value);
	bool WriteA32BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, UNSIGNED_INTEGER32 value);
	bool ReadA32BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, UNSIGNED_INTEGER32* value);
	bool ReadLowLevelServoAndMotorCommands(SrvoAndMotorCommands* motors);
	bool SystemReset();
	bool ReadLastRC5Frame(RC5Frame* frame);
	bool ReadDigitalInputs(DigitalInputs* inputs);
	bool WriteLowLevelServoAndMotorCommands(SrvoAndMotorCommands motors);


	
private:
	bool ExecuteInstruction( RAW_BYTE* response,int numberOfInstructionBytes,int numberOfResponseBytes,const char* name);


};

#endif
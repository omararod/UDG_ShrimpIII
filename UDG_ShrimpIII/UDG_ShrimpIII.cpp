#include "UDG_ShrimpIII.h"
#ifndef __linux
#include "SerialPort.h"
#else
#include "SerialPortLinux.h"
#endif

UDG_ShrimpIII::UDG_ShrimpIII(const char* serialPortName)
{
#ifdef __linux
	int descriptor = OpenSerialPort(serialPortName);
	ConfigurePort(descriptor);
#else
	OpenSerialPort(serialPortName);
#endif
	velocity = 0;
	steeringAngle = 0;
	
}
UDG_ShrimpIII::~UDG_ShrimpIII()
{
	CloseSerialPort();
}

//Generic instruction to read/write instruction byte codes
bool UDG_ShrimpIII::ExecuteInstruction(RAW_BYTE* instructionBuffer,int numberOfInstructionBytes,int numberOfResponseBytes,const char* name)
{
	
	RAW_BYTE originalID = instructionBuffer[0];
	if (WriteToSerial(instructionBuffer,numberOfInstructionBytes) != numberOfInstructionBytes)
	{
		printf("Error while writing instruction: %s\n",name);
		return false;
	}
	instructionBuffer[0] = 0xFF;
	if( ReadFromSerial(instructionBuffer,numberOfResponseBytes) != numberOfResponseBytes)
	{
	
		printf("Error while receiving response from instruction: %s\n",name);
		return false;
	}
	if(originalID != instructionBuffer[0])
	{
		printf("Unexpexted response from instruction %s: %2X\n",name,instructionBuffer[0]);
		return false;
	}

	return true;
}

//This command doesn't have any effect. It can be used as a "ping" to ensure that communication with the robot is working. It can also be used in case of loss of synchronization in the command stream: sending a string of 0x00 bytes at least as long as the longest command available ensures that the command following the 0x00 bytes will be interpreted correctly.
bool UDG_ShrimpIII::NoOperation()
{
	shrimpBuffer[0] = 0x00;
	return ExecuteInstruction(shrimpBuffer,1,1,"'No Operation'");

}
//This command returns the version of the controller's firmware. The result is stored in firmwareVersion
//Buffer receives |0x01| Major (8) |Minor (8) |Patch (8)|
//For example, for version 1.4.7, Major=1, Minor=4 and Patch=7.
bool UDG_ShrimpIII::ReadFirmwareVersion()
{
	bool ret;
	shrimpBuffer[0] = 0x01;
	ret =  ExecuteInstruction(shrimpBuffer,1,4,"'Read Firmware Version'");
	firmwareVersion.major = shrimpBuffer[1];
	firmwareVersion.minor = shrimpBuffer[2];
	firmwareVersion.patch = shrimpBuffer[3];
	return ret;
}
//This command connects power to the motor and servo controllers, enabling the movement of the robot.
bool UDG_ShrimpIII::TurnRobotOn()
{
	shrimpBuffer[0] = 0x02;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Turn On'");

}
//This command removes power from the motor and servo controllers. Power consumption goes down to a minimum, so it can be used to conserve power when the robot is not moving
bool UDG_ShrimpIII::TurnRobotOff()
{
	shrimpBuffer[0] = 0x03;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Turn Off'");

}
//This command sets new target values for the linear velocity and steering angle of the robot. The motor and servo controllers will be reprogrammed immediately to reach the new targets
//velocity (-127..127): speed of the rear wheel. The value -127 is full speed backward, 127 is full speed forward, and 0 is stopped
//angle (-90..90): angle of the rear wheel relative to the straight line position, in degrees. Negative values are angles to the right (meaning that the robot will turn to the left), positive values to the left.
//updates velocity and steering angle members
bool UDG_ShrimpIII::SetVelocityAndSteeringAngle(SIGNED_BYTE velocity, SIGNED_BYTE angle)
{
	SIGNED_BYTE adjustedAngle = angle;
	shrimpBuffer[0] = 0x04;
	shrimpBuffer[1] = velocity;
	this->velocity = velocity;
	//make sure the angle is not out of boundries
	if(adjustedAngle < -90)
	{
		adjustedAngle = -90;
		printf("Angle out of bounds, adjusting to 90 degrees\n");
	}
	if(adjustedAngle > 90)
	{
		adjustedAngle = 90;
		printf("Angle out of bounds, adjusting to -90 degrees\n");
	}
	steeringAngle = adjustedAngle;
	shrimpBuffer[2] = adjustedAngle;

	return ExecuteInstruction(shrimpBuffer,3,1,"'Set Velocity and steering angle'");
}
//This command returns the current velocity and steering angle of the robot, and can be used for example to log the commands that are sent to the robot with the infrared remote control.
//Buffer receives |0x05| Velocity (8)| Steering angle (8) |
bool UDG_ShrimpIII::ReadVelocityAndSteeringAngle()
{
	bool ret;
	shrimpBuffer[0] = 0x05;
	ret =  ExecuteInstruction(shrimpBuffer,1,3,"'Read Velocity and Steering Angle'");
	velocity = shrimpBuffer[1];
	steeringAngle = shrimpBuffer[2];
	return ret;
}
//This command stops all motors on the robot (i.e. sets their speeds to zero). The steering servo positions remain unchanged
bool UDG_ShrimpIII::EmergencyStop()
{
	shrimpBuffer[0] = 0x06;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Emergency Stop'");
}
//This command reads and returns the encoder values of all six wheels, allowing to make simple odometry calculations or to get a feeling of how far the robot has travelled.
//The wheelEncoders property will be updated
//See struct WheelEncoders definition for more details
//WheelEncoders* encoders: pointer to a struct used as a return value, this is a copy of the value at wheelEncoders property. Set to null if it´s not important to you
//Buffer Receives| 0x07| Value F (32) |Value FL (32) |Value FR (32)| Value BL (32)| Value BR (32)| Value B (32) |
bool UDG_ShrimpIII::ReadWheelEncoderValues(WheelEncoders* encoders )
{
	bool ret;
	shrimpBuffer[0] = 0x07;
	ret = ExecuteInstruction(shrimpBuffer,1,25,"'Read Wheel Encoder Values'");
	if( ret )
	{
		if(encoders)memcpy(encoders,&shrimpBuffer[1],sizeof(WheelEncoders));
		memcpy(&wheelEncoders,&shrimpBuffer[1],sizeof(WheelEncoders));
	}
	return ret;
}
//This command returns the current status of the robot.
//Updates the robotStatus property and, if provided, stores the same value at status, a pointer to a RobotStatus struct
bool UDG_ShrimpIII::ReadRobotStatus(RobotStatus* status)
{
	bool ret;
	shrimpBuffer[0] = 0x08;
	ret = ExecuteInstruction(shrimpBuffer,1,2,"'Read Robot Status'");
	if( ret )
	{
		if(status)memcpy(status,&shrimpBuffer[1],sizeof(RobotStatus));
		memcpy(&robotStatus,&shrimpBuffer[1],sizeof(RobotStatus));
	}
	return ret;
}

//This command reads the current voltage of the battery on the robot. It is updated about once per second, so reading it more often will return the same value between updates.
//Returns the voltage value and updates the voltage property. Return value is 0 if an error occurs
//Buffer receives | 0x09| Battery voltage (8) |
float UDG_ShrimpIII::ReadBatteryVoltage()
{
	bool ret;
	shrimpBuffer[0] = 0x09;
	ret = ExecuteInstruction(shrimpBuffer,1,2,"'Read Battery Voltage'");
	if(ret)
	{
		voltage = 0.0625f * (float)(shrimpBuffer[1]);
		return voltage;
	}
	else
	{
		return 0;
	}


}

//This command returns the current status of the power supply controller. It reports any problems the power supply controller could identify, for example low battery states. The security thresholds satisfy the following condition:
//Vmin < Vsecure < Vlow < Vmax
//Updates the powerSupplyStatus property and, if provided, stores the same value at status, a pointer to a powerSupplyStatus struct
bool UDG_ShrimpIII::ReadPowerSupplyStatus(PowerSupplyStatus* status)
{
	bool ret;
	shrimpBuffer[0] = 0x0A;
	ret = ExecuteInstruction(shrimpBuffer,1,2,"'Read Power Supply Status'");
	if( ret )
	{
		if(status)memcpy(status,&shrimpBuffer[1],sizeof(PowerSupplyStatus));
		memcpy(&powerSupplyStatus,&shrimpBuffer[1],sizeof(PowerSupplyStatus));
	}
	return ret;
}
//This command disables the infrared remote control decoder. It can be useful if the robot is controlled through RS-232 and there are infrared transmitters in the vincinity
bool UDG_ShrimpIII::DisableInfraredRemoteControl()
{
	shrimpBuffer[0] = 0x0B;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Disable Infrared Remote Control'");

}
//This command enables the infrared remote control decoder
bool UDG_ShrimpIII::EnableInfraredRemoteControl()
{
	shrimpBuffer[0] = 0x0C;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Enable Infrared Remote Control'");

}
//On power-up, the power supply controller is programmed to make various sounds when the battery level goes below certain thresholds. This command mutes those sounds.
bool UDG_ShrimpIII::MutePowerSupplyBuzzer()
{
	shrimpBuffer[0] = 0x0D;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Mute Power Supply Buffer'");

}
//This command enables batter-level sounds of the power supply controller. This is the default power-up state.
bool UDG_ShrimpIII::UnmutePowerSupplyBuzzer()
{
	shrimpBuffer[0] = 0x0E;
	return ExecuteInstruction(shrimpBuffer,1,1,"'Unmute Power Supply Buffer'");

}
//This command writes an 8-bit register on an I2C module. The master acts as a pass-through
bool UDG_ShrimpIII::WriteAn8BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, RAW_BYTE value)
{
	shrimpBuffer[0] = 0x0F;
	shrimpBuffer[1] = moduleAddress;
	shrimpBuffer[2] = registerAddress;
	shrimpBuffer[3] = value;
	return ExecuteInstruction(shrimpBuffer,4,1,"'Write an 8 bit register on an I2C module'");
}
//This command reads an 8-bit register from an I2C module. The master acts as a pass-through.
//The value is stored at value, a pointer to a RAW_BYTE variable
bool UDG_ShrimpIII::ReadAn8BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, RAW_BYTE* value)
{
	shrimpBuffer[0] = 0x10;
	shrimpBuffer[1] = moduleAddress;
	shrimpBuffer[2] = registerAddress;
	
	bool ret = ExecuteInstruction(shrimpBuffer,3,2,"'Read an 8 bit register on an I2C module'");

	if(ret && value)
	{*value = shrimpBuffer[1];}

	return ret;
	 
}
//This command writes a 32-bit register on an I2C module. The master acts as a pass-through
bool UDG_ShrimpIII::WriteA32BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, UNSIGNED_INTEGER32 value)
{
	shrimpBuffer[0] = 0x11;
	shrimpBuffer[1] = moduleAddress;
	shrimpBuffer[2] = registerAddress;
	memcpy(&shrimpBuffer[3],&value,sizeof(UNSIGNED_INTEGER32));
	return ExecuteInstruction(shrimpBuffer,7,1,"'Write a 32 bit register on an I2C module'");
}
//This command reads a 32-bit register from an I2C module. The master acts as a pass-through.
//The value is stored at value, a pointer to a UNSIGNED_INTEGER32 variable
bool UDG_ShrimpIII::ReadA32BitRegisterOnAnI2CModule(RAW_BYTE moduleAddress, RAW_BYTE registerAddress, UNSIGNED_INTEGER32* value)
{
	shrimpBuffer[0] = 0x12;
	shrimpBuffer[1] = moduleAddress;
	shrimpBuffer[2] = registerAddress;
	
	bool ret = ExecuteInstruction(shrimpBuffer,3,5,"'Read a 32 bit register on an I2C module'");

	if(ret && value)
		memcpy(value,&shrimpBuffer[1],sizeof(UNSIGNED_INTEGER32));

	return ret;
	 
}
//This command reads and returns the low-level commands sent to the servos and motors. It should only be used as a debugging aid.
//buffer receives |0x13 |Servo F (16)| Servo B (16)| Motor F (32)| Motor L (32)| Motor R (32)| Motor B (32)| 
bool UDG_ShrimpIII::ReadLowLevelServoAndMotorCommands(SrvoAndMotorCommands* motors )
{
	shrimpBuffer[0] = 0x13;
	
	
	bool ret = ExecuteInstruction(shrimpBuffer,1,21,"'Read Low Level Servo and Motor Commands'");

	if(ret)
	{
			if(motors)memcpy(&motors,&shrimpBuffer[1],sizeof(SrvoAndMotorCommands));
			memcpy(&servoAndMotorCommands,&shrimpBuffer[1],sizeof(SrvoAndMotorCommands));
		
	}
	return ret;
	 
}

//This command resets the main controller and re-initializes the complete system.
bool UDG_ShrimpIII::SystemReset()
{
	shrimpBuffer[0] = 0x14;
	return ExecuteInstruction(shrimpBuffer,1,1,"'System Reset'");

}

bool UDG_ShrimpIII::ReadLastRC5Frame(RC5Frame* frame)
{
	shrimpBuffer[0] = 0x15;
	
	
	bool ret = ExecuteInstruction(shrimpBuffer,1,3,"'Read Last RC5 frame'");

	if(ret && frame)
		memcpy(&frame,&shrimpBuffer[1],sizeof(RC5Frame));

	return ret;
}

//This command returns the current state of the digital inputs to the microcontroller. The emergency stop input is active-low, i.e. the robot is stopped when nESTOP=0.
bool UDG_ShrimpIII::ReadDigitalInputs(DigitalInputs* inputs)
{
	shrimpBuffer[0] = 0x16;
	
	bool ret = ExecuteInstruction(shrimpBuffer,1,2,"'Read Digital Inputs'");

	if(ret )
	{
		if(inputs)memcpy(&inputs,&shrimpBuffer[1],sizeof(RC5Frame));
		memcpy(&digitalInputs,&shrimpBuffer[1],sizeof(RC5Frame));
	}

	return ret;
}
//This command writes low-level commands directly to the servos and motors, bypassing the robot model. No checking is done on the values, such as out-of-bounds conditions, so it might be possible to damage the hardware by writing bad values. This command should normally not be used.
bool UDG_ShrimpIII::WriteLowLevelServoAndMotorCommands(SrvoAndMotorCommands motors)
{
	shrimpBuffer[0] = 0x16;
	memcpy(&shrimpBuffer[1],&motors,sizeof(SrvoAndMotorCommands));

	return  ExecuteInstruction(shrimpBuffer,21,1,"'Read Last RC5 frame'");;
}

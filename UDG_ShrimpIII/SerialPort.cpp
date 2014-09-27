#ifdef __linux
#include "SerialPortLinux.h"
#include <errno.h>
#include <string>
int portDescriptor;

int OpenSerialPort(const char* name)
{    portDescriptor = open(name, O_RDWR | O_NOCTTY);
	if(portDescriptor<0) 
	{  	printf("Error opening port\n");
		printf("Port: %s\n",name);
	}
	else
	{
		printf("Succesfully opened %s\n",name);
	}
	
	return portDescriptor;
}



/*-----------------------------------------------------------------------------------------
int ConfigurePort(int portDescriptor)
PortDescriptor: An int descriptor for the selected port

Sets all the needed configurations for serial communication. Returns -1 if something
goes wrong
--------------------------------------------------------------------------------------------
*/
int ConfigurePort(int portDescriptor)
{
int returnValue;
struct termios configuration;
	//===================Configure port===================
//more info http://homepages.cwi.nl/~aeb/linux/man2html/man3/termios.3.html
/*IGNBRK:	Ignore break conditions
  BRKINT:	flushes the queue when a brake is received
  ICRNL:	Translate CR to NL
  INLCR:	Translate NL to CR 
  PARMRK:	Mark parity errors
  INPCK:	Parity checking
  ISTRIP:	Strip off eight bit
    IXON:	XON/XOFF flow control
*/
//Disables those options
memset(&configuration,0,sizeof(configuration));
configuration.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
configuration.c_oflag=0;
/*ECHO:	echo input characters
 ECHONL: echoes the NL characer if ICANON is set
 ICANON: Canonical mode
 IEXTEN: inplementation-defined input processing
 ISIG:	generate INTR, QUIT, SUSP, or DSUSP signals
*/
configuration.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
/*CSIZE: Character size mask. Values are CS5, CS6, CS7, or CS8
 PARENB: parity generation on output and parity checking for input.
	CS8: See CSIZE
*/
configuration.c_cflag &= ~(CSIZE | PARENB);
configuration.c_cflag |= CS8;
/*VMIN: minimum number of bytes received before read() returns
  VTIME: Tenths of second before consider a read() as finished
*/
configuration.c_cc[VMIN]  = 1;
configuration.c_cc[VTIME] = 0;

returnValue = cfsetispeed(&configuration, B57600) < 0 || cfsetospeed(&configuration, B57600);
if(returnValue<0)
{
	printf("Error setting serial port speed\n");
	return returnValue;
}
/*TCSANOW: Apply immediately
*/
returnValue = tcsetattr(portDescriptor, TCSANOW, &configuration);
if(returnValue < 0)
 { printf("Error configuring serial port\n");}
printf("Serial Port Succesfully configured\n");
return returnValue;
}

/*-----------------------------------------------------------------------------------------
int WriteToSerial(int portDescriptor,char* buffer, int numberOfBytes)

writes numberOfBytes bytes from buffer to portDescriptor port
--------------------------------------------------------------------------------------------
*/
int WriteToSerial(unsigned char* buffer, int numberOfBytes)
{
	int returnValue;
	returnValue=write(portDescriptor,buffer,numberOfBytes);
	if(returnValue<0)
	{printf("error writing to serial port\n");}
	return returnValue;
}
int ReadFromSerial(unsigned char* buffer,int numberOfBytes)
{
int returnValue;
	returnValue=read(portDescriptor,buffer,numberOfBytes);
	if(returnValue<0)
	{printf("error reading from serial port\n"); printf("%i\n",errno);}
	return returnValue;
}

void CloseSerialPort()
{
	if(portDescriptor)
	{
		close(portDescriptor);
	}
}


#else
#include "SerialPort.h"

HANDLE portDescriptor;
DCB deviceControlBlock;
void OpenSerialPort(const char* name)
{
	
	portDescriptor = CreateFile(name,
				GENERIC_READ|GENERIC_WRITE,
				0,0,OPEN_EXISTING,0,0);
	if(portDescriptor == INVALID_HANDLE_VALUE)
	{
		printf("Failed opening %ls\n", name);
		return;
	}

	FillMemory(&deviceControlBlock,sizeof(DCB),0);
	deviceControlBlock.DCBlength = sizeof(DCB);
	if(BuildCommDCB("57600,n,8,1",&deviceControlBlock))
	{
		printf("Successfully configured DCB!\n");
		
	}
	else
	{
		printf("Something went wrong while building the DCB\n");
		return;
	}

	if (!SetCommState(portDescriptor,&deviceControlBlock))
	{
		printf("Error configuring port\n");
		return;
	}
	else
	{
		printf("Serial port ready\n");
	}
		

}
void CloseSerialPort()
{
	if(portDescriptor)
	{
		CloseHandle(portDescriptor);
	}
}

int WriteToSerial(unsigned char buffer[], int bytesToWrite)
{
	DWORD writtenBytes=0;
	if(portDescriptor != NULL)
	WriteFile(portDescriptor,buffer,bytesToWrite,&writtenBytes,NULL);
	return (int)(writtenBytes);
}

int ReadFromSerial(unsigned char buffer[], int bytesToRead )
{
	DWORD readBytes = 0;
	if(portDescriptor != NULL)
	ReadFile(portDescriptor,buffer,bytesToRead,&readBytes,NULL);
	return (int)(readBytes);
}
#endif

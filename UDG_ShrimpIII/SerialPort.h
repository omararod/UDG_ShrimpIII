#ifndef SERIALPORT_H
#define SERIALPORT_H

#ifndef __linux

#include <Windows.h>
#include <stdio.h>


	
	

	void OpenSerialPort(const char* name);
	void CloseSerialPort();
	int ReadFromSerial(unsigned char buffer[], int bytesToRead );
	int WriteToSerial(unsigned char buffer[], int bytesToWrite);

#endif
	

#endif

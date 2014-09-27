#ifndef SERIALPORTLINUX_H
#define SERIALPORTLINUX_H
#define ERROR -1

#include <termios.h>	//UNIX API for terminal I/O
#include <fcntl.h>	// Constant declarations for termios functions
#include <unistd.h> 	//close() function
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <string.h>	//needed for memset
#include <thread>
#include <stdarg.h>

using namespace std;

int OpenSerialPort(const char* portName);
int ConfigurePort(int portDescriptor);
int WriteToSerial(unsigned char* buffer,int numberOfBytes);
int ReadFromSerial(unsigned char* buffer,int numberOfBytes);
void CloseSerialPort();
int AutoOpenPort();


#endif

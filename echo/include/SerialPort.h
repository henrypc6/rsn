#ifndef SERIALPORT_H
#define SERIALPORT_h

#include <string.h>
#include <unistd.h>
#include <stdint.h>

int OpenSerialPort(char *);
void SendSwitchData(int);
void SetSwitches(char**);
int ReadSonarReturnData(int, unsigned char*);
int Data_Check(unsigned char*);


#endif

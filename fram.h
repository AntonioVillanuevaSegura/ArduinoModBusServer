//Antonio Villanueva Segura MODBUS FRAM
#ifndef FRAM_H_
#define FRAM_H_

#include <Arduino.h>
#include <Wire.h>
#include <FastCRC.h>


//Write from 0 to ( 256 * 8) = 2048 bytes
void writeI2CByte(uint16_t mem_addr, byte data);

//Reading between 0 to ( 256 * 8) = 2048 bytes
byte readI2CByte(uint16_t mem_addr);

//DEBUG funnc. Read the entire FM24CL16 FRAM from 0 to (256 * 8) = 2048 bytes, shows values other than 0
void readFM24CL16();

//Reset all FM24CL16 FRAM memory to 0
void resetFM24CL16 ();

//DUMP  array ( uint8_t [size]to FRAM mem_addr  , address=  (0x50-051 data) (0x52-0x53 holding Reg.) (0x55-0x57 coils)
void arrayToFRAM(uint8_t address,uint8_t *matriz, int size);

//Dump FM24CL16B bank (0x5n)  to uint8_t [] array  address=  (0x50-051 data) (0x52-0x53 holding Reg.) (0x55-0x57 coils
//...Continuos reading memory addresss = i2c address
void FRAMToArray(uint8_t address,uint8_t *matriz, int size);

// Returns CRC16 Modbus from uint16_t buffer[256] https://crccalc.com/
uint16_t crc16 (uint8_t *buffer,int size);

// Debug Print, See array uint16_t [] buffer
void seeArray (uint16_t *buf ,size_t size);

#endif //FRAM_H_

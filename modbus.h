//Antonio Villanueva Segura MODBUS 
#ifndef MODBUS_H_
#define MODBUS_H_

#include "Adafruit_EEPROM_I2C.h"
#include <ArduinoModbus.h>
#include "expander.h"
#include <Arduino.h>

//2048 /16 =128 strac 0x0000 0x0080 
//INs memory not physical
//#define N_HOLDING_REGISTERS 2
#define N_HOLDING_REGISTERS 128
#define HOLD_REG_ADDRESS 0x00

//OUTs memory not physical 0x0000 0x07FF
#define N_COILS 2048
//#define N_COILS 16
#define COIL_ADDRESS 0x00

//Real physical digital inputs ,Depends on hardware implementation ,in our case strac has 8 ...
#define N_INPUTS 1
#define INPUTS_ADDRESS 0x00

//Define for Serial Debug Print
#define READ_COILS 0
#define READ_HOLDING_REG 1
#define READ_INPUTS 2

//Une fonction pour compléter le nombre de 0's dans une expression BINARIE  base=8 ou 16 p.e
String zeroComplement(String number, int base);

//Lire les entrées MCP2317 réelles et les copier sur modbus "discreteInputWrite"
int writeInputs(ModbusTCPServer *modbusTCPServer ,int address,byte input);

//Récupère l'état des OUTs de la mémoire et les écrit dans les coils modbus
void setCoils(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp ,int address,int n, uint8_t value);

//We configure the modbus system Holding Registers , Coils , Inputs ...
void configureModbus(ModbusTCPServer *modbusTCPServer);

//The modbus client activates the coils, then we activate the expander bus outputs the electrical relays
void setOutputs(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp,Adafruit_EEPROM_I2C *i2ceeprom, int address );

//Print a 16-bit register through the serial port
void printRegister(String reg,int address,int i);

//Imprime en-tête hexadécimal, pour indiquer la position du bit FEDCBA9876543210  
void printIndex ();

//Prints through the tty serial output each memory Register location works with printRegister() and printIndex()
void debugRegister (int type, ModbusTCPServer *modbusTCPServer,int address, int n);

#endif //MODBUS_H_

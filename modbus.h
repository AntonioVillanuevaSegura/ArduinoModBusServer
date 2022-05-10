//Antonio Villanueva Segura MODBUS 
#ifndef MODBUS_H_
#define MODBUS_H_

#include "Adafruit_EEPROM_I2C.h"
#include <ArduinoModbus.h>
#include "expander.h"
#include <Arduino.h>

#define N_HOLDING_REGISTERS 2
#define HOLD_REG_ADDRESS 0x00

#define N_COILS 16
#define COIL_ADDRESS 0x00

#define N_INPUTS 1
#define INPUTS_ADDRESS 0x00

//Imprime un en-tête hexadécimal, pour indiquer la position du bit
void printIndex ();

//Une fonction pour compléter le nombre de 0's dans une expression BINARIE  base=8 ou 16 p.e
String zeroComplement(String number, int base);

//Lire les entrées MCP2317 réelles et les copier sur modbus "discreteInputWrite"
int writeInputs(ModbusTCPServer *modbusTCPServer ,int address,byte input);

//afficher les coils  "coilRead" au niveau de la console série
void getCoils(ModbusTCPServer *modbusTCPServer,int address,int n);

//Récupère l'état des OUTs de la mémoire et les écrit dans les coils modbus
void setCoils(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp ,int address,int n, uint8_t value);

//Lire les entrées modbus "discreteInputRead" à afficher sur le bus série ttyACM0
void getInputs(ModbusTCPServer *modbusTCPServer,int address,int n);

//afficher le holding Register "holdingRegisterRead" au niveau de la console série
void getHoldingRegister(ModbusTCPServer *modbusTCPServer,int address);

//We configure the modbus system Holding Registers , Coils , Inputs ...
void configureModbus(ModbusTCPServer *modbusTCPServer);

//The modbus client activates the coils, then we activate the expander bus outputs the electrical relays
void setOutputs(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp,Adafruit_EEPROM_I2C *i2ceeprom, int address );
#endif //MODBUS_H_

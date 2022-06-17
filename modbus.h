//Antonio Villanueva Segura MODBUS  https://www.arduino.cc/reference/en/libraries/arduinomodbus/
#ifndef MODBUS_H_
#define MODBUS_H_

#include "Adafruit_EEPROM_I2C.h"
#include <ArduinoModbus.h>
#include "expander.h"
#include <Arduino.h>

//*************************************************************************************** 
//2048 /16 =128 strac 0x0000 0x0080 
//INs memory not physical
#define N_HOLDING_REGISTERS 128
#define HOLD_REG_ADDRESS 0x00


//Real physical digital inputs ,Depends on hardware implementation ,in our case strac has 8 ...
#define N_INPUTS 1
#define INPUTS_ADDRESS 0x00

//Define for Serial Debug Print

#define READ_HOLDING_REG 1
#define READ_INPUTS 2

//*************************************************************************************** 


//dumps a uint16_t buffer[128] to the Holding Registers ,normally used at boot to retrieve information from the FRAM.
void arrayToHoldingRegisters (ModbusTCPServer *modbusTCPServer,int address, int n,uint16_t *buffer,size_t size);

//We configure the modbus system Holding Registers , Coils , Inputs ...
void configureModbus(ModbusTCPServer *modbusTCPServer);

//Lire les entrées MCP2317 réelles et les copier sur modbus "discreteInputWrite"
int writeInputs(ModbusTCPServer *modbusTCPServer ,int address,byte input);

//The modbus client activates the relays, then we activate the expander bus outputs physical outputs of the expander bus MCP23017 ...8 Coils or Relays
void setRelays(DFRobot_MCP23017 *mcp,uint8_t *outs );

//Récupère l'état des OUTs de la mémoire et les écrit dans les coils modbus
//void setCoils(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp ,int address,int n, uint8_t value);

//The modbus client activates the coils, then we activate the expander bus outputs the electrical relays
//void setRelays(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp,Adafruit_EEPROM_I2C *fram, int address );

//Une fonction pour compléter le nombre de 0's dans une expression BINARIE  base=8  00000000 ou 16 p.e 0000000000000000
String zeroComplement(String number, int base);

//Print a 16-bit register through the serial port
void printRegister(String reg,int address,int i);

//Imprime en-tête hexadécimal, pour indiquer la position du bit dans le registre FEDCBA9876543210  
void printIndex ();

//Prints through the tty serial output each memory Register location works with printRegister() and printIndex()
/*type =READ_HOLDING_REG or READ_INPUTS 
 * address =HOLD_REG_ADDRESS or INPUTS_ADDRESS 0x00
 * n =N_HOLDING_REGISTERS or N_INPUTS
 * buffer = uint16_t buffer[16]
 * size == 128 or  (sizeof(buffer) / sizeof (buffer[0]) )
 */
void debugRegister (int type, ModbusTCPServer *modbusTCPServer,int address, int n,uint16_t *buffer,size_t size);


#endif //MODBUS_H_

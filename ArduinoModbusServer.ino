/*
  Modbus TCP server  Antonio Villanueva Segura
  https://www.arduino.cc/reference/en/libraries/arduinomodbus/
  https://tutoduino.fr/tutoriels/arduino-modbus-tcp/
  
  FM24CL16B 2048 x 8 =16Kbit bits https://www.mouser.fr/datasheet/2/100/CYPR_S_A0011126722_1-2541208.pdf

video test ARDUINO MKR WIFI 1010
https://www.youtube.com/watch?v=NBX_xE0cCc8&feature=youtu.be

ARDUINO NANO 
A5 SCL Yellow
A4 SDA Green
D10 CS SPI
D11 MOSI SPI
D12 MISO SPI
D13 SCK SPI
 
ARCELI W5500 SPI 
LAN Ethernet Module réseau TCP IP STM32 Interface 3.3V 5V 
https://www.amazon.fr/gp/product/B09DYFLMJG/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1
GND                                         GND 
GND                                         VCC --> +3V3 arduino nano
MOSI --> D11 arduino nano                   VCC --> +3V3 arduino nano
SCK -->  D13 arduino nano                   NC
CS  -->  D10 arduino nano (D5) MKR_1010     RST
INT    
  
*/
#define I2C_ADDRESS 0x20 //mcp23017 expander
byte input=0; //S'il y a des changements aux entrées réelles (i2c MCP23017), affichez-les sur la console série.ttyACM0

//*************************************************************************************** 

#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>

#include "fram.h"
#include "expander.h" 
#include "modbus.h"
#include "Adafruit_EEPROM_I2C.h"

//Choisissez la carte à compiler

#define ARDUINO_MKR_WIFI_1010
//#define ARDUINO_NANO 

#define SERIAL_SPEED 9600
#define IP_ADDRESS 192,168,6,69
#define MAC_ADDRESS 0xA8, 0x61, 0x0A, 0xAE, 0x7A, 0x69
EthernetServer ethServer(502);//Server on port 502
ModbusTCPServer modbusTCPServer; //TCP modbus server

DFRobot_MCP23017 mcp(Wire, /*addr =*/I2C_ADDRESS);//Expander MCP 23017

uint16_t buffer[128]={0}; //Buffer 128*16=2048 ....tmp . Intermediary buffer between the Fram and the modbus registers

Adafruit_EEPROM_I2C fram;

#define LED LED_BUILTIN
//***************************************************************************************  
void setup() {

  pinMode(LED_BUILTIN, OUTPUT); //LED 
  
  // MAC address of the Ethernet Shield
  byte mac[] = { MAC_ADDRESS };
   
  // IP address of the Ethernet Shield
  IPAddress ip(IP_ADDRESS);
   
  Serial.begin(SERIAL_SPEED); 

  while (!Serial){} //Wait for Serial Debug , If there is no serial port it blocks

  #if defined ARDUINO_MKR_WIFI_1010
    Serial.println ("ARDUINO_MKR_WIFI_1010");
    Ethernet.init(5) ; //PIN 5 on MKR WIFI 1010  
  #else
    Serial.println ("ARDUINO_NANO");  
    Ethernet.init (10);//D6 on arduino ..... CS pin
  #endif 
  
  Ethernet.begin(mac, ip);  // Start Ethernet connection

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {//Check ETH hardware 
    Serial.println(" Error ETH hardware W5500  ");
    while (true) { 
      digitalWrite(LED_BUILTIN, not digitalRead (LED_BUILTIN) ); //Problem link
      delay(500);
      }
  }

  ethServer.begin();//Start Ethernet Server
   
  // Starts the Modbus TCP server
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (true) { delay(1);} //Error Modbus TCP server loop nothing
  } else {Serial.println ("Modbus TCP Server started ");}

  delay(500);

  //Declare modbus struct holding_registers , coils 
  configureModbus(&modbusTCPServer);

  Serial.print("Ethernet Modbus on ");
  Serial.println (ip);
  
  expanderSetup ( &mcp );//mcp23017 setup i2c expander I/O
  
  //i2c FRam FM24C16B 2077748 setup
  if (fram.begin(0x50)) { Serial.println("Found I2C FRam");}
  else {Serial.println ("I2C FRam not found ");while (true){ };}


  //Reads Fram memory to buffer[128] and the MODBUS memory 
  //First Fram --to --> uint16_t buffer [128] 
  
  FRAMToArray(0x50,(uint8_t *) buffer,  256); //128*x2 =256 uint8_t   Read Fram to uint16_t (uint8_t ) buffer  array

  //Reads CRC16 from FRAM 0x51   .... byte readI2CByte(uint16_t mem_addr)
  uint16_t crcFram= (readI2CByte(257)) << 8; //CRC16 from FRAM
  crcFram |= readI2CByte(256); 

  //Calcul CRC16 from uint16_t buffer[128] pr uint8_t [256]
  uint16_t crcBuffer=crc16 ((uint8_t *) buffer,  256);//Calcul CRC16 . NOTE If the buffer uint16_t measures 128 in uint8_t measures 256

  //Compare the CRC16 retrieved from the FRAM with the calculated CRC16 .If is different, buffer [] reset to 0 as well as the CRC value.
  Serial.print ("CRC from Fram = 0x");Serial.print(crcFram,HEX);Serial.print (" , CRC calculated = 0x");Serial.println(crcBuffer,HEX);//DEBUG PRINT
  
  if (crcFram != crcBuffer){ //The CRC16 retrieved from the Fram does not coincide with the calculated CRC16 value
    Serial.println ("Wrong CRC16 value ! Reset Fram ");
    resetFM24CL16 ();//Reset FRAM to 0's
    
    //CRC16 value for 0's in 0x50 255 bytes and CRC16
    writeI2CByte(256, 0xBF);//CRC16 H precalculated for 256 zero values
    writeI2CByte(257, 0x64); //CRC16 L precalculated for 256 zero values
    
    }else {Serial.println ("CRC 16 is ok "); }
   
  //Writes uint16_t [128] ( or uint8_t[256] cast ) array to Holding Registers
  arrayToHoldingRegisters ( &modbusTCPServer,HOLD_REG_ADDRESS, N_HOLDING_REGISTERS ,buffer, ( sizeof (buffer) /sizeof (buffer[0])) );

  //Writes buffer[0] (holdingRegisters) to Relays 
  setRelays( &mcp , ( (uint8_t*) &buffer[0] ) );//Physical outputs of the expander bus MCP23017 ...8 Coils or Relays buffer[0]

}
//*************************************************************************************** 
void loop() {

  if (Ethernet.linkStatus() == LinkOFF) {//Detect ETH cable 
    Serial.println("Ethernet cable is not connected ! ");
    while (Ethernet.linkStatus() == LinkOFF){}
    Serial.println("Ethernet cable is OK ! ");
    }

 
  // listen for incoming clients
  EthernetClient client = ethServer.available();
   
  if (client) { // a new client connected

    clientModbus(&client);
    /*
    
    digitalWrite(LED_BUILTIN, HIGH ); //Client LED Client connected
    Serial.println("\nClient connected !");
 
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
 
   if(client.connected()) {// loop while the client is connected

    modbusTCPServer.poll();//Poll Modbus TCP request while client connected

    //Prints through the tty serial output each memory Register location
    //void debugRegister (int type, ModbusTCPServer *modbusTCPServer,int address, int n);

    debugRegister (READ_HOLDING_REG, &modbusTCPServer,HOLD_REG_ADDRESS, N_HOLDING_REGISTERS,buffer,  (sizeof(buffer) / sizeof (buffer[0])) );
    debugRegister (READ_INPUTS, &modbusTCPServer,INPUTS_ADDRESS, N_INPUTS,buffer,(sizeof(buffer) / sizeof (buffer[0])));  
     
    arrayToFRAM(0x50,(uint8_t *) buffer,  256);// Write uint16_t buffer ( uint8_t) buffer array to Fram 

  //Calcul CRC16 from uint16_t buffer[128] pr uint8_t [256]
   uint16_t crc=crc16 ((uint8_t *) buffer,  256);//Calcul CRC16 . NOTE If the buffer uint16_t measures 128 in uint8_t measures 256
      
   Serial.print("CRC16 = 0x ");Serial.println (crc, HEX); //DEBUG print CRC16

  //Save CRC16 in FRAM 0x51 ... memory abs 256 ,257 , relative 0x51:00 0x51:01
  writeI2CByte(256, crc);
  writeI2CByte(257, ( crc>>8) );   
  
  setRelays( &mcp , ( (uint8_t*) &buffer[0] ) );//Physical outputs of the expander bus MCP23017 ...8 Coils or Relays buffer[0]

    }
 
    Serial.println("\nClient disconnected");
    digitalWrite(LED_BUILTIN, LOW ); //LED OFF CLIENT Client disconnected
    */
  }

  //S'il y a des changements aux entrées réelles (i2c MCP23017), affichez-les sur la console série.ttyACM0
  if ((readPort(&mcp,'A'))!= input ){
    input=readPort(&mcp,'A');
    writeInputs(&modbusTCPServer,INPUTS_ADDRESS,input);
    //getInputs(&modbusTCPServer,INPUTS_ADDRESS,8);
    debugRegister (READ_INPUTS, &modbusTCPServer,INPUTS_ADDRESS, N_INPUTS,buffer, (sizeof(buffer) / sizeof (buffer[0]))); 
  
  }
  delay(500);
 
}
//*************************************************************************************** 
void clientModbus(EthernetClient *client ){

    digitalWrite(LED_BUILTIN, HIGH ); //Client LED Client connected
    Serial.println("\nClient connected !");
 
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(*client);
 
   if((*client).connected()) {// loop while the client is connected

    modbusTCPServer.poll();//Poll Modbus TCP request while client connected

    //Prints through the tty serial output each memory Register location
    //void debugRegister (int type, ModbusTCPServer *modbusTCPServer,int address, int n);

    debugRegister (READ_HOLDING_REG, &modbusTCPServer,HOLD_REG_ADDRESS, N_HOLDING_REGISTERS,buffer,  (sizeof(buffer) / sizeof (buffer[0])) );
    debugRegister (READ_INPUTS, &modbusTCPServer,INPUTS_ADDRESS, N_INPUTS,buffer,(sizeof(buffer) / sizeof (buffer[0])));  
     
    arrayToFRAM(0x50,(uint8_t *) buffer,  256);// Write uint16_t buffer ( uint8_t) buffer array to Fram 

  //Calcul CRC16 from uint16_t buffer[128] pr uint8_t [256]
   uint16_t crc=crc16 ((uint8_t *) buffer,  256);//Calcul CRC16 . NOTE If the buffer uint16_t measures 128 in uint8_t measures 256
      
   Serial.print("CRC16 = 0x ");Serial.println (crc, HEX); //DEBUG print CRC16

  //Save CRC16 in FRAM 0x51 ... memory abs 256 ,257 , relative 0x51:00 0x51:01
  writeI2CByte(256, crc);
  writeI2CByte(257, ( crc>>8) );   
  
  setRelays( &mcp , ( (uint8_t*) &buffer[0] ) );//Physical outputs of the expander bus MCP23017 ...8 Coils or Relays buffer[0]

    }
 
    Serial.println("\nClient disconnected");
    digitalWrite(LED_BUILTIN, LOW ); //LED OFF CLIENT Client disconnected

  
  
}

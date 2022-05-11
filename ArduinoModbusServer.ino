/*
  Modbus TCP server  Antonio Villanueva Segura
  https://www.arduino.cc/reference/en/libraries/arduinomodbus/
  https://tutoduino.fr/tutoriels/arduino-modbus-tcp/

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

#include "expander.h" 
#include "modbus.h"
#include "Adafruit_EEPROM_I2C.h"

//Choisissez la carte à compiler

#define ARDUINO_MKR_WIFI_1010
//#define ARDUINO_NANO 

#define SERIAL_SPEED 9600
#define IP_ADDRESS 192,168,1,69
#define MAC_ADDRESS 0xA8, 0x61, 0x0A, 0xAE, 0x7A, 0x69
EthernetServer ethServer(502);//Server on port 502
ModbusTCPServer modbusTCPServer; //TCP modbus server

DFRobot_MCP23017 mcp(Wire, /*addr =*/I2C_ADDRESS);//Expander MCP 23017


Adafruit_EEPROM_I2C i2ceeprom;

#define LED LED_BUILTIN
//***************************************************************************************  
void setup() {


  pinMode(LED_BUILTIN, OUTPUT); //LED 
  
  // MAC address of the Ethernet Shield
  byte mac[] = { MAC_ADDRESS };
   
  // IP address of the Ethernet Shield
  IPAddress ip(IP_ADDRESS);
   
  Serial.begin(SERIAL_SPEED); 

  while (!Serial){} //Wait for Serial Debug 

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
   
  // Start the Modbus TCP server

  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (true) { delay(1);} //Error Modbus TCP server loop nothing
  } else {Serial.println ("Modbus TCP Server started ");}

  delay(500);

  //Declare modbus struct holding_registers , coils 
  configureModbus(&modbusTCPServer);

  Serial.print("Ethernet Modbus on ");
  Serial.println (ip);
  
  expanderSetup ( &mcp );//mcp23017 setup
  
  //i2c EEPROM FM24C16B 2077748 setup
  if (i2ceeprom.begin(0x50)) { Serial.println("Found I2C EEPROM");}
  else {Serial.println ("I2C EEPROM not found ");while (true){ };}

  //Read i2c EEPROM FM24C16B OUTs
  setCoils(&modbusTCPServer ,&mcp,0x00,8, i2ceeprom.read(0x0));
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
    digitalWrite(LED_BUILTIN, HIGH ); //Client LED Client connected
    Serial.println("\nClient connected !");
 
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
 
   if(client.connected()) {// loop while the client is connected

    modbusTCPServer.poll();//Poll Modbus TCP request while client connected

    //Afficher les données sur la console série
    printIndex ();//Index HEXA ,Affiche un index indiquant les positions des bits 
    
    getHoldingRegister(&modbusTCPServer,HOLD_REG_ADDRESS);//Read first holding register on address 0x0000
    getHoldingRegister(&modbusTCPServer,HOLD_REG_ADDRESS+1);//Read Second holding register on address 0x0001 
    
    getCoils(&modbusTCPServer,COIL_ADDRESS,N_COILS); //Read Coils Relays OUTPUTs
    getInputs(&modbusTCPServer,INPUTS_ADDRESS,N_INPUTS);//Read Real Inputs   

    setOutputs(&modbusTCPServer,&mcp,&i2ceeprom,COIL_ADDRESS );//Le client a accede, nous mettons à jour les sorties , les relais
    
    }
 
    Serial.println("\nClient disconnected");
    digitalWrite(LED_BUILTIN, LOW ); //LED OFF CLIENT Client disconnected
  }

  //S'il y a des changements aux entrées réelles (i2c MCP23017), affichez-les sur la console série.ttyACM0
  if ((readPort(&mcp,'A'))!= input ){
    input=readPort(&mcp,'A');
    writeInputs(&modbusTCPServer,INPUTS_ADDRESS,input);
    getInputs(&modbusTCPServer,INPUTS_ADDRESS,8);
  
  }
  delay(500);
 
}

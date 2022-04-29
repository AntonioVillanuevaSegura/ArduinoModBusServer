/*
  Modbus TCP server 
  https://www.arduino.cc/reference/en/libraries/arduinomodbus/
  https://tutoduino.fr/tutoriels/arduino-modbus-tcp/

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


#include <SPI.h>
#include <Ethernet.h>
//#include <Ethernet2.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

EthernetServer ethServer(502);//Server on port 502
ModbusTCPServer modbusTCPServer; //TCP modbus server


#define LED LED_BUILTIN
//***************************************************************************************  
void setup() {

  pinMode(LED_BUILTIN, OUTPUT); //LED 
  
  // MAC address of the Ethernet Shield
  byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x7A, 0x69 };
   
  // IP address of the Ethernet Shield
  IPAddress ip(192, 168, 6, 69);
   
  Serial.begin(9600); 

  Ethernet.init(5) ; //PIN 5 on MKR WIFI 1010
  //Ethernet.init (10);//D6 on arduino ..... CS pin

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

  configureModbus(0x00 ,1);

  Serial.print("Ethernet Modbus on ");
  Serial.println (ip);

}
//*************************************************************************************** 
void loop() {

  if (Ethernet.linkStatus() == LinkOFF) {//Detect ETH cable 
    Serial.println(F("Ethernet cable is not connected ! "));
    while (Ethernet.linkStatus() == LinkOFF){}
    Serial.println(F("Ethernet cable is OK ! "));
    }

 
  // listen for incoming clients
  EthernetClient client = ethServer.available();
   
  if (client) { // a new client connected
    digitalWrite(LED_BUILTIN, HIGH ); //Client LED
    Serial.println("Client connected !");
 
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
 
   if(client.connected()) {// loop while the client is connected

      modbusTCPServer.poll();//Poll Modbus TCP request while client connected
    Serial.println (modbusTCPServer.holdingRegisterRead(0x0000));//long holdingRegisterRead(int address);
    }
 
    Serial.println("Client disconnected");
    digitalWrite(LED_BUILTIN, LOW ); //LED OFF  CLIENT
  }
 
}

//*************************************************************************************** 
void configureModbus(int address ,int nb){
   if ( modbusTCPServer.configureHoldingRegisters(address, nb)){Serial.println (F("Error on create Holding Reg"));}
   else{Serial.println ("Holding Registers OK");}

}

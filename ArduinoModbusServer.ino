/*
  Modbus TCP server  Antonio Villanueva Segura
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

#define N_HOLDING_REGISTERS 2
#define HOLD_REG_ADDRESS 0x00
#define N_COILS 16
#define COIL_ADDRESS 0x00
//*************************************************************************************** 
#include <SPI.h>
#include <Ethernet.h>
//#include <Ethernet2.h>
//#include <ArduinoRS485.h> // ArduinoModbus depends  ArduinoRS485 library
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

  while (!Serial){} //Wait for Serial Debug 

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

  //Declare modbus struct holding_registers , coils 
  configureModbus(HOLD_REG_ADDRESS ,N_HOLDING_REGISTERS,COIL_ADDRESS,N_COILS);

  Serial.print("Ethernet Modbus on ");
  Serial.println (ip);
  delay(50);
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
    digitalWrite(LED_BUILTIN, HIGH ); //Client LED
    Serial.println("Client connected !");
 
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
 
   if(client.connected()) {// loop while the client is connected

      modbusTCPServer.poll();//Poll Modbus TCP request while client connected
    //Serial.println (modbusTCPServer.holdingRegisterRead(0x0000),BIN);//long holdingRegisterRead(int address);

    getHoldingRegister(HOLD_REG_ADDRESS);//Read first holding register on address 0x0000
    getHoldingRegister(HOLD_REG_ADDRESS+1);//Read Second holding register on address 0x0001 
    getCoils(COIL_ADDRESS,N_COILS);  
    getInputs(COIL_ADDRESS,N_COILS);  
    }
 
    Serial.println("Client disconnected");
    digitalWrite(LED_BUILTIN, LOW ); //LED OFF  CLIENT
  }
 
}
//*************************************************************************************** 
String longToString (long reg){
  //Long 4 bytes 4*8=32
  String tmp=String(reg,BIN);

  if (tmp.length() <16){ //0 -31 bits
    String zeros="";
      for (int i=0; i<16-tmp.length();i++){
        zeros+='0';
      }

    zeros+=tmp;
    
  }
  
  
}

//*************************************************************************************** 
void getCoils(int address,int n){//int coilRead(int address);
  Serial.print ("\nCoils Read  ( ");
  Serial.print (address);
  Serial.print (" to ");
  Serial.print (address+n);  
  Serial.print (" ) = ");
  
  int valeur(-1);

  for (int coil=n-1;coil>=0;coil--){
    //Serial.print (coilRead(address+coil);)
    valeur=modbusTCPServer.coilRead(address+coil);
    //if (valeur){Serial.println (valeur);}
    Serial.print(valeur);
  }
  Serial.println ("");
  
}
//*************************************************************************************** 
void getInputs(int address,int n){//int discreteInputRead(int address);
  Serial.print ("\nInputs Read ( ");
  Serial.print (address);
  Serial.print (" to ");
  Serial.print (address+n);  
  Serial.print (" ) = ");
  
  int valeur(-1);

  for (int input=n-1;input>=0;input--){
    valeur=modbusTCPServer.discreteInputRead(address+input);
    Serial.print (valeur);
    //if (valeur){Serial.println (valeur);}

  }
  Serial.println ("");
  
}

//*************************************************************************************** 
void getHoldingRegister(int address){
  Serial.print ("\nholdingRegisterRead (");
  Serial.print (address);
  Serial.print (") = ");
  long reg = modbusTCPServer.holdingRegisterRead(address);//long holdingRegisterRead(int address);
 
  String tmp= String(reg,BIN);//Base BIN
  String zeros="";
  for ( unsigned i=0 ; i< (16-tmp.length());i++ ){zeros+='0';} //Print 0's
  zeros+=tmp;

  tmp="";
  for (unsigned i=0;i<zeros.length();i++){
    tmp+=zeros[i];
  }
 Serial.println (zeros);
  
}
//*************************************************************************************** 
void configureModbus(int hold_reg_address ,int holding_registers,int coil_address, int coils){
    //Holding Registers
   if ( ! modbusTCPServer.configureHoldingRegisters(hold_reg_address, holding_registers)){Serial.println (F("Error on create Holding Reg"));}
   else{Serial.println ("Holding Registers OK");}

    //Coils

    //int configureCoils(int startAddress, int nb);
   if ( ! modbusTCPServer. configureCoils( coil_address, coils)){Serial.println (F("Error on create Coils"));}
   else{Serial.println ("Coils OK");}    

    //Inputs
    //int configureDiscreteInputs(int startAddress, int nb);
  if ( ! modbusTCPServer. configureDiscreteInputs( coil_address, coils)){Serial.println (F("Error on create Inputs"));}
  else{Serial.println ("Inputs OK");} 

}

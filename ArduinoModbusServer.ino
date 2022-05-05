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

#define N_HOLDING_REGISTERS 2
#define HOLD_REG_ADDRESS 0x00

#define N_COILS 16
#define COIL_ADDRESS 0x00

#define N_INPUTS 1
#define INPUTS_ADDRESS 0x00

//*************************************************************************************** 
#include "expander.h" 
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>

EthernetServer ethServer(502);//Server on port 502
ModbusTCPServer modbusTCPServer; //TCP modbus server

DFRobot_MCP23017 mcp(Wire, /*addr =*/I2C_ADDRESS);//Expander MCP 23017

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
  configureModbus();

  Serial.print("Ethernet Modbus on ");
  Serial.println (ip);

  delay(50);
  
  expanderSetup ( &mcp );//mcp23017 set Up
  
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
    digitalWrite(LED_BUILTIN, HIGH ); //Client LED Client connected
    Serial.println("\nClient connected !");
 
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
 
   if(client.connected()) {// loop while the client is connected

    modbusTCPServer.poll();//Poll Modbus TCP request while client connected

    //Afficher les données sur la console série
    printIndex ();//Index HEXA ,Affiche un index indiquant les positions des bits 
    getHoldingRegister(HOLD_REG_ADDRESS);//Read first holding register on address 0x0000
    getHoldingRegister(HOLD_REG_ADDRESS+1);//Read Second holding register on address 0x0001 
    getCoils(COIL_ADDRESS,N_COILS); //Read Coils 
    getInputs(INPUTS_ADDRESS,N_INPUTS);//Read Real Inputs   

    setOutputs(COIL_ADDRESS );//Le client a accede, nous mettons à jour les sorties , les relais
    
    }
 
    Serial.println("\nClient disconnected");
    digitalWrite(LED_BUILTIN, LOW ); //LED OFF CLIENT Client disconnected
  }

  //S'il y a des changements aux entrées réelles (i2c MCP23017), affichez-les sur la console série.ttyACM0
  if ((readPort(&mcp,'A'))!= input ){
    input=readPort(&mcp,'A');
    writeInputs(INPUTS_ADDRESS,input);
    getInputs(INPUTS_ADDRESS,8);
  
  }
  delay(500);
 
}
//*************************************************************************************** 
//Une fonction pour compléter le nombre de 0's dans une expression BINARIE  base=8 ou 16 p.e
String zeroComplement(String number, int base){
  String zeros="";
  for ( unsigned i=0 ; i< (base - ( number.length()) );i++ ){zeros+='0';} //Print 0's
  zeros+=number;
  
  number="";
  for (unsigned i=0;i<zeros.length();i++){
  number+=zeros[i];
  }
  //Serial.println (zeros);
  return zeros;
  
}

//*************************************************************************************** 
//Lire les entrées MCP2317 réelles et les copier sur modbus "discreteInputWrite"
int writeInputs(int address,byte input){ 
  modbusTCPServer.discreteInputWrite(address,input );
}
//*************************************************************************************** 
//afficher les coils  "coilRead" au niveau de la console série
void getCoils(int address,int n){//int coilRead(int address);
  Serial.print ("\nCoils Read  ( ");
  Serial.print (address);
  Serial.print (" to ");
  Serial.print (address+n);  
  Serial.print (" ) = ");
  
  int valeur(-1);

  for (int coil=n-1;coil>=0;coil--){
    valeur=modbusTCPServer.coilRead(address+coil);
    Serial.print(valeur);
  }
  Serial.println ("");
  
}
//*************************************************************************************** 
//Lire les entrées modbus "discreteInputRead" à afficher sur le bus série ttyACM0
void getInputs(int address,int n){

  Serial.print ("\nInputs Read    (Real)   = "); 
  int valeur= modbusTCPServer.discreteInputRead(address);
  Serial.println (zeroComplement(String(valeur,BIN),16));
}

//*************************************************************************************** 
//Imprime un en-tête hexadécimal, pour indiquer la position du bit
void printIndex (){
  Serial.print("                          ");  
  for (int i=15;i>=0;i--){
    Serial.print (i,HEX);
  }
}

//*************************************************************************************** 
//afficher le holding Register "holdingRegisterRead" au niveau de la console série
void getHoldingRegister(int address){
  Serial.print ("\nholdingRegisterRead (");
  Serial.print (address);
  Serial.print (") = ");
  long reg = modbusTCPServer.holdingRegisterRead(address);//long holdingRegisterRead(int address);
 
  String tmp= String(reg,BIN);//Base BIN

  Serial.println (zeroComplement(tmp, 16));
}
//*************************************************************************************** 
//We configure the modbus system Holding Registers , Coils , Inputs ...
void configureModbus(){

    //Holding Registers
   if ( ! modbusTCPServer.configureHoldingRegisters(HOLD_REG_ADDRESS, N_HOLDING_REGISTERS)){Serial.println (F("Error on create Holding Reg"));}
   else{Serial.println ("Holding Registers OK");}

    //Coils
    //int configureCoils(int startAddress, int nb);
   if ( ! modbusTCPServer. configureCoils( COIL_ADDRESS, N_COILS)){Serial.println (F("Error on create Coils"));}
   else{Serial.println ("Coils OK");}    

    //Inputs
    //int configureDiscreteInputs(int startAddress, int nb);
  if ( ! modbusTCPServer. configureDiscreteInputs( INPUTS_ADDRESS,N_INPUTS)){Serial.println (F("Error on create Inputs"));}
  else{Serial.println ("Inputs OK");} 

}
//*************************************************************************************** 
//The modbus client activates the coils, then we activate the expander bus outputs the electrical relays
void setOutputs(int address ){//Physical outputs of the expander bus MCP23017 ...8 Coils or Relays

  uint8_t value=0x00;//8 coils

  for (int coil=7;coil>=0;coil--){//read 8 modbus coils (coilRead) 
    value =value<<1;//rotate << left
    //value += ( (modbusTCPServer.coilRead(address+coil)>0) ? 1 : 0 ) ; //Read actual coil
    value += modbusTCPServer.coilRead(address+coil) ; //Read coil
    delay(25);
  }

  //We send the byte read from the modbus to the expander bus
  //void setPort(DFRobot_MCP23017 *mcp,uint8_t *value)
  setPort(&mcp,&value); //Set MCP23017 Physical Relays 
}

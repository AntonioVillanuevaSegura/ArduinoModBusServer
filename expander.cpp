//Antonio Villanueva Segura Gestion Bus expander i2c mcp23017
#include "expander.h"

byte valeur=0;
void expanderSetup (DFRobot_MCP23017 *mcp){
    while((*mcp).begin() != 0){//Espera inicializar el mcp23017
    Serial.println(F("i2c error !"));
    delay(1000);
  }

  Serial.println ("OK bus expander init MCP23017");
  /*pinMode function is used to set the pin mode of the module
  Parameter pin, the available parameter is shown below:
  eGPA0  eGPA1  eGPA2  eGPA3  eGPA4  eGPA5  eGPA6  eGPA7  eGPA
   0       1      2      3      4      5      6      7
  eGPB0  eGPB1  eGPB2  eGPB3  eGPB4  eGPB5  eGPB6  eGPB7  eGPB
   8       9      10     11     12     13     14     15
  Parameter mode, can be set to: INPUT, OUTPUT, INPUT_PULLUP mode (internal 100KΩ pull-up resistor)
  */
 // mcp.pinMode(/*pin = */mcp.eGPA7, /*mode = */OUTPUT);
  /*Set all Group GPIOA pins to output*/
  //mcp.pinMode(/*pin = */mcp.eGPA, /*mode = */OUTPUT);
  
   (*mcp).pinMode(/*pin = */(*mcp).eGPB, /*mode = */OUTPUT);//PORT B OUT
   (*mcp).pinMode(/*pin = */(*mcp).eGPA, /*mode = */INPUT);//PORT A IN  
}

byte readPort(DFRobot_MCP23017 *mcp,char port){

  if (port=='A'){
    (*mcp).pinMode(/*pin = */(*mcp).eGPA, /*mode = */INPUT);//PORT A IN  
    return (*mcp).digitalRead(/*pin = */(*mcp).eGPA);
  }

  //(*mcp).pinMode(/*pin = */(*mcp).eGPB, /*mode = */INPUT);//PORT B IN  
  return (*mcp).digitalRead(/*pin = */(*mcp).eGPB);
  
}

//void setPort(DFRobot_MCP23017 *mcp,byte *value){
void setPort(DFRobot_MCP23017 *mcp,uint8_t *value){

  (*mcp).pinMode(/*pin = */(*mcp).eGPB, /*mode = */OUTPUT);//PORT B OUTPUT 
  (*mcp).digitalWrite(/*pin = */(*mcp).eGPB,*value);
  
}

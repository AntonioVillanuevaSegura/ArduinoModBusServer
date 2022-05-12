//Antonio Villanueva Segura MODBUS 
#include "modbus.h"


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
int writeInputs(ModbusTCPServer *modbusTCPServer ,int address,byte input){ 
  (*modbusTCPServer).discreteInputWrite(address,input );
}

//*************************************************************************************** 
//Récupère l'état des OUTs de la mémoire et les écrit dans les coils modbus
void setCoils(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp,int address,int n, uint8_t value){//int coilWrite(int address, uint8_t value);
   Serial.print ("\nWrite OUT's from i2c EEPROM = 0x");
   Serial.print (value,HEX);
   Serial.print (" ");Serial.println (value,BIN);

  //We send the byte read from the modbus to the expander bus
  //void setPort(DFRobot_MCP23017 *mcp,uint8_t *value)
  setPort(mcp,&value); //Set MCP23017 Physical Relays 
  
  for (int coil=n-1;coil>=0;coil--){
    uint8_t puiss= (1 << coil);
    uint8_t state=( (puiss & value) >0 ? 1:0 );
    (*modbusTCPServer).coilWrite(address+coil, state);
    delay(10);    
  }

}

//*************************************************************************************** 
//We configure the modbus system Holding Registers , Coils , Inputs ...
void configureModbus(ModbusTCPServer *modbusTCPServer){

    //Holding Registers
   if ( ! (*modbusTCPServer).configureHoldingRegisters(HOLD_REG_ADDRESS, N_HOLDING_REGISTERS)){Serial.println (F("Error on create Holding Reg"));}
   else{Serial.print(N_HOLDING_REGISTERS); Serial.println (" Holding Registers OK");}

    //Coils
    //int configureCoils(int startAddress, int nb);
   if ( ! (*modbusTCPServer). configureCoils( COIL_ADDRESS, N_COILS)){Serial.println (F("Error on create Coils"));}
   else{Serial.print(N_COILS);Serial.println (" Coils OK");}    

    //Inputs
    //int configureDiscreteInputs(int startAddress, int nb);
  if ( ! (*modbusTCPServer). configureDiscreteInputs( INPUTS_ADDRESS,N_INPUTS)){Serial.println (F("Error on create Inputs"));}
  else{Serial.print(N_INPUTS);Serial.println (" Inputs OK");} 

}
//*************************************************************************************** 
//The modbus client activates the coils, then we activate the expander bus outputs the electrical relays
void setOutputs(ModbusTCPServer *modbusTCPServer,DFRobot_MCP23017 *mcp,Adafruit_EEPROM_I2C *i2ceeprom, int address ){//Physical outputs of the expander bus MCP23017 ...8 Coils or Relays

  uint8_t value=0x00;//8 coils

  for (int coil=7;coil>=0;coil--){//read 8 modbus coils (coilRead) 
    value =value<<1;//rotate << left
    //value += ( (modbusTCPServer.coilRead(address+coil)>0) ? 1 : 0 ) ; //Read actual coil
    value += (*modbusTCPServer).coilRead(address+coil) ; //Read coil
    delay(25);
  }

  //We send the byte read from the modbus to the expander bus
  //void setPort(DFRobot_MCP23017 *mcp,uint8_t *value)
  setPort(mcp,&value); //Set MCP23017 Physical Relays 

 //Set the i2c EEPROM FM24C16B 
 (*i2ceeprom).write(0x0, value);//write 8 out's coils
  
}

//*************************************************************************************** 
//Print a 16-bit register through the serial port
void printRegister(String reg,int address,int i){
  reg=zeroComplement(reg, 16);

  Serial.print(reg);
  reg=" ["+ String (address+i,HEX)+"]\t";          
  Serial.print (reg);

  //if (i%64==0){Serial.println();}//Ligne
    
}

//*************************************************************************************** 
//Imprime en-tête hexadécimal, pour indiquer la position du bit FEDCBA9876543210 
void printIndex (){
  for (int i=15;i>=0;i--){Serial.print (i,HEX);}
}

//*************************************************************************************** 
//Prints through the tty serial output each memory Register location works with printRegister() and printIndex()
void debugRegister (int type, ModbusTCPServer *modbusTCPServer,int address, int n){
  switch (type){
    case (READ_COILS):{Serial.println("\nCOILS");break;}
    case (READ_HOLDING_REG):{Serial.println("\nHOLDING REGISTER");break;}
    case (READ_INPUTS):{Serial.println("\nREAL PHYSICAL INPUTS");break; }        
  }
  
  //Write Hex index
  for (int i=0 ;i< ( (n>4)?4:n ) ;i++){printIndex ();Serial.print('\t');}
  Serial.println ();//\n

  String tmp;//Auxiliary variable stores 16 bits in a string

  switch (type){//type READ_COILS 0 ,READ_HOLDING_REG 1 ,READ_INPUTS 2
    
    case (READ_COILS):{    
      for (int i=n-1;i>=0;i--){//Loop through all READ_COILS memory  locations
        tmp+=(*modbusTCPServer).coilRead(address+i);
        if (i%16==0) {printRegister(tmp,address, i);tmp="";}
        if (i%64==0){Serial.println();}//Ligne        
      }        
      break;
    }
    
    case (READ_HOLDING_REG):{    
      for (int i=n-1;i>=0;i--){//Loop through all READ_COILS memory  locations
        long reg = (*modbusTCPServer).holdingRegisterRead(address+i);//long holdingRegisterRead(int address);      
        tmp= String(reg,BIN);//Base BIN
        printRegister(tmp,address, i);
        if (i%4==0){Serial.println();}//Ligne         
      }
      break;    
    }
    
    case (READ_INPUTS):{
      for (int i=n-1;i>=0;i--){//Loop through all READ_INPUTS memory  locations      
        int valeur= (*modbusTCPServer).discreteInputRead(address+i);   
        tmp=zeroComplement(String(valeur,BIN),16);
        printRegister(tmp,address, i);
        if (i%4==0){Serial.println();}//Ligne         
      }   
      break;        
    }
  }
}


  
  

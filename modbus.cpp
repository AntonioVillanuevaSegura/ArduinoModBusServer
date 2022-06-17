//Antonio Villanueva Segura MODBUS  https://www.arduino.cc/reference/en/libraries/arduinomodbus/
#include "modbus.h"

//*************************************************************************************** 
//dumps a uint16_t buffer[128] to the Holding Registers ,normally used at boot to retrieve information from the FRAM.
void arrayToHoldingRegisters (ModbusTCPServer *modbusTCPServer,int address, int n,uint16_t *buffer,size_t size){

  for (int i=0;i<size;i++){//Walk over uint16_t buffer[128]
    //int holdingRegisterWrite(int address, uint16_t value);
    (*modbusTCPServer).holdingRegisterWrite(address+i, (*buffer) );//Write holdingRegister in address + offset
    buffer++;
    
  }
  
}

//*************************************************************************************** 
//We configure the modbus system Holding Registers , Coils , Inputs ...
void configureModbus(ModbusTCPServer *modbusTCPServer){

    //Holding Registers
   if ( ! (*modbusTCPServer).configureHoldingRegisters(HOLD_REG_ADDRESS, N_HOLDING_REGISTERS)){Serial.println (F("Error on create Holding Reg"));}
   else{Serial.print(N_HOLDING_REGISTERS); Serial.println (" Holding Registers OK");}
  
    //Inputs
    //int configureDiscreteInputs(int startAddress, int nb);
  if ( ! (*modbusTCPServer). configureDiscreteInputs( INPUTS_ADDRESS,N_INPUTS)){Serial.println (F("Error on create Inputs"));}
  else{Serial.print(N_INPUTS);Serial.println (" Inputs OK");} 

}

//*************************************************************************************** 
//Lire les entrées MCP2317 réelles et les copier sur modbus "discreteInputWrite"
int writeInputs(ModbusTCPServer *modbusTCPServer ,int address,byte input){ 
  (*modbusTCPServer).discreteInputWrite(address,input );
}

//*************************************************************************************** 
//The modbus client activates the relays, then we activate the expander bus outputs physical outputs of the expander bus MCP23017 ...8 Coils or Relays
void setRelays(DFRobot_MCP23017 *mcp,uint8_t *outs ){//Physical outputs of the expander bus MCP23017 ...8 Coils or Relays
  setPort(mcp,outs);
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
//Print a 16-bit register through the serial port
void printRegister(String reg,int address,int i){
  reg=zeroComplement(reg, 16);//Complement the number of 0's up to 16

  Serial.print(reg);//Print REG value
  reg=" ["+ String (address+i,HEX)+"]\t";          
  Serial.print (reg);//Print HEX address
   
}

//*************************************************************************************** 
//Imprime en-tête hexadécimal, pour indiquer la position du bit dans le registre FEDCBA9876543210 
void printIndex (){for (int i=15;i>=0;i--){Serial.print (i,HEX);}}

//*************************************************************************************** 
//Prints through the tty serial output each memory Register location works with printRegister() and printIndex()
void debugRegister (int type, ModbusTCPServer *modbusTCPServer,int address, int n,uint16_t *buffer,size_t size){
  switch (type){
    case (READ_HOLDING_REG):{Serial.println("\nHOLDING REGISTER");break;}
    case (READ_INPUTS):{Serial.println("\nREAL PHYSICAL INPUTS");break; }        
  }
  
  //Write Hex index
  for (int i=0 ;i< ( (n>4)?4:n ) ;i++){printIndex ();Serial.print('\t');}
  Serial.println ();//\n

  String tmp;//Auxiliary variable stores 16 bits in a string

  switch (type){//type READ_COILS 0 ,READ_HOLDING_REG 1 ,READ_INPUTS 2
    
    case (READ_HOLDING_REG):{    
      buffer+=size-1;//starts at the end of buffer i=n-1 !
      
      for (int i=n-1;i>=0;i--){//Loop through all HOLDING_REG memory  locations
        long reg = (*modbusTCPServer).holdingRegisterRead(address+i);//long holdingRegisterRead(int address);    

        //Write uint16_t [128] register
        *buffer=reg;//  > Write in buffer[128]= 16 holding_register

       // tmp= String(reg,BIN);//Base BIN
        tmp= String(*buffer,BIN);//Base BIN
        printRegister(tmp,address, i);
        if (i%4==0){Serial.println();}//Ligne            
        buffer--;//Buffer ++ move one position
                 
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

/*
 * Antonio Villanueva Segura
 * FM24CL16 FRAM management continuous memory paging and CRC16 MODBUS
 * 
 * https://www.mouser.fr/datasheet/2/100/CYPR_S_A0010744052_1-2541108.pdf
 * 
 * Access by memory location 0 a 256*8=2048 
 * The base i2c address is  0b1010 
 * After rotating to the left is 0b1010 <<3 =0x50 The base i2c address
 * add the least significant bits A2 A1 A0 to the i2c base address 0x50 
 * every block  0x5N ( where N is = 1,2,3,4,5,6,7) contains 256 Bytes, and bits =256 x 8 x 8 = 16384
 * i2c address 0x51 0x52 0x53 0x54 0x55 0x56 0x57
 * 
 *                    A7  A6  A5  A4  A3  A2  A1  A0
 * 256 x0 0-255       0   1   0   1   0   0   0   0   0x50
 * 256 x1 256-511     0   1   0   1   0   0   0   1   0x51
 * 256 x2 512-764     0   1   0   1   0   0   1   0   0x52
 * 256 x3 768-1023    0   1   0   1   0   0   1   1   0x53
 * 256 x4 1024-1279   0   1   0   1   0   1   0   0   0x54
 * 256 x5 1280-1535   0   1   0   1   0   1   0   1   0x55
 * 256 x6 1536-1791   0   1   0   1   0   1   1   0   0x56
 * 256 x7 1792- 2047  0   1   0   1   0   1   1   1   0x57
 * 
 */

/*
#include <Wire.h>
#include <FastCRC.h>
*/
#include "fram.h"

//Base I2C address 0b1010 << 3 = 0x50
#define ADDR (0b1010 << 3)  

/*Block memory 8 blocks with address (0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57) 
 * which make a total of  256x8=2048 bytes 
 * and  256 x 8 x 8 = 16384 bits */
 
#define MEMORY_BLOCK 256 

//String address="";
String data="";
uint8_t buf[256];//buf_array to  FM24CL16 and FM24CL16 to buf_array


/**************************************************************************************************************/
//Write from 0 to ( 256 * 8) = 2048 bytes
void writeI2CByte(uint8_t mem_addr, byte data){

  Serial.print ("ADDR 0x");Serial.print (ADDR | ( mem_addr /MEMORY_BLOCK ) ,HEX);Serial.print ( ", 0x");Serial.println ( (mem_addr-(MEMORY_BLOCK *  (mem_addr/MEMORY_BLOCK))),HEX );
  Wire.beginTransmission (ADDR | ( mem_addr /MEMORY_BLOCK ) ); //Slave Address 0x5n , 0x50 | A2 A1 A0
  Wire.write(mem_addr-(MEMORY_BLOCK *  (mem_addr/MEMORY_BLOCK) ));//Address in memory bank 0 -256
  Wire.write(data);//Data byte
  Wire.endTransmission(false);

}

/**************************************************************************************************************/
//Reading between 0 to ( 256 * 8) = 2048 bytes
byte readI2CByte(uint8_t mem_addr){

  byte data=0x00;
  uint8_t address(ADDR); //Base Address 0x50
  address += ( mem_addr /MEMORY_BLOCK );//A2 A1 A0 =  add to 0x50  -> 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07

 
  Wire.beginTransmission( address & 0xFF) ;//Slave Address 0x5n , 0x50 | A2 A1 A0
  Wire.write(mem_addr-(MEMORY_BLOCK *  (mem_addr/MEMORY_BLOCK) ));//Address in memory bank 0 -256
  Wire.endTransmission();

  Wire.requestFrom(address, 1); //retrieve 1 returned byte
  delay(1);

  if(Wire.available()){data = Wire.read();}
  return data;
  
  
}

/**************************************************************************************************************/
//DEBUG funnc. Read the entire FM24CL16 FRAM from 0 to (256 * 8) = 2048 bytes, shows values other than 0
void readFM24CL16(){
  Serial.println ("Analyze memory values , other than 0");
  uint8_t data;
    for (int mem=0 ;mem<MEMORY_BLOCK*8;mem++){//Loop over fram memory
      data =readI2CByte(mem);
      if (data>0){ //If value data !=0
        Serial.print ("Address ( ");Serial.print (mem);
        Serial.print (" ) = ");Serial.println (data); 
      }
      data=0;
    }
}
/**************************************************************************************************************/
//Reset all FM24CL16 FRAM memory to 0
void resetFM24CL16 (){
   for (int mem=0 ;mem<MEMORY_BLOCK*8;mem++){//Loop over fram memory
    Serial.print ("Reset mem pos. = ");Serial.println (mem);
    writeI2CByte(mem, 0);
   }
}
/**************************************************************************************************************/
//DUMP  array ( uint8_t [size]to FRAM mem_addr  , address=  (0x50-051 data) (0x52-0x53 holding Reg.) (0x55-0x57 coils)
void arrayToFRAM(uint8_t address,uint8_t *matriz, int size){

  Wire.beginTransmission (address ); //Slave Address 0x5n , 0x50 | A2 A1 A0
  Wire.write(0x00);//First Address in FRAM bank 0 -256
  
  while (size>0){//All bank 0-256
    Wire.write( *matriz );//Data byte
    matriz++;//Pointer ++
    size--;//Array size --
  }

  Wire.endTransmission(false);
}

/**************************************************************************************************************/
//Dump FM24CL16B bank (0x5n)  to uint8_t [] array  address=  (0x50-051 data) (0x52-0x53 holding Reg.) (0x55-0x57 coils
//...Continuos reading memory addresss = i2c address
void FRAMToArray(uint8_t address,uint8_t *matriz, int size){

  int n=0;
  Wire.beginTransmission (address ); //Slave Address 0x5n | A2 A1 A0 0x50 0x51 0x52 0x53 0x54 0x55 0x56 0x57
  Wire.write(0x00);//First Address in FRAM bank 0 -256
  Wire.endTransmission(false);  
  
   Wire.requestFrom(address, size);    // Request size bytes from slave device number address  0x5n
    // Slave may send less than requested
    while(Wire.available()) {
      *matriz=Wire.read();
      matriz++; //Pointer to receive array
    }

    delay(500);
}
/**************************************************************************************************************/
// Returns CRC16 Modbus from uint16_t buffer[256] 
uint16_t crc16 (uint8_t *buffer,int size){
  FastCRC16 CRC16;
  return CRC16.modbus( buffer,size);
}
/**************************************************************************************************************/
// Debug Print, See array uint16_t []
void seeArray (uint16_t *buf ,size_t size){
  for (int i=15;i>=0;i--){Serial.print (i,HEX);} //en-tête hexadécimal
  
  Serial.println ();
  for (int i=0 ;i<size;i++){

   // Serial.print(*buf,BIN);Serial.print(" [");Serial.print(i,DEC),Serial.println("]");

  //Read bit from word
  for (int b=0;b<16;b++){Serial.print(bitRead(*buf,b));}
  
   Serial.print(" [");Serial.print(i,DEC),Serial.println("]");
    ++buf;    
  }  
}

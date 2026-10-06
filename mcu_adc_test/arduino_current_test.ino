/************************************************************************
*
* Test of the Pmod
*
*************************************************************************
* Description: Pmod_AD1
* The result of A / D conversion of channel A0 is displayed on the serial monitor.
* Material
* 1. Arduino Uno
* 2. Pmod AD1 
*
************************************************************************/

#define CS 10 // Assignment of the CS pin
#include <SPI.h> // call library

int i;
byte recu[3]; // storage of data of the module
int X;

void setup()
{
 Serial.begin(115200); // initialization of serial communication
 SPI.begin(); // initialization of SPI port
 SPI.setDataMode(SPI_MODE0); // configuration of SPI communication in mode 0
 SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz/8 = 2MHz
 pinMode(CS, OUTPUT);
}

void loop()
{
 digitalWrite(CS, LOW); // activation of CS line
 delayMicroseconds(10);
 for (i=0;i<2;i=i+1)
   {
   recu[i] = SPI.transfer(0); // send 2 data to retrieve the converted value on 2 bytes
  //  delayMicroseconds(10);
   }
 digitalWrite(CS, HIGH); // deactivation of CS line

 X = recu[1]; // X is in a 12 bit format
 X |= (recu[0] << 8);

// X = (recu[1] | (recu[0] << 8)) >> 4; 

 for (i=0;i<2;i=i+1) // Send in serial monitor
   {
  //  Serial.print("i");
  //  Serial.print(i);
  //  Serial.print("=");
  //  Serial.print(recu[i]);
  //  Serial.print('\t'); // tabulation
   }
//  Serial.print("X=");
 Serial.println(X);
 delay(1);
}
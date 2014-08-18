/*
 * Program Code for Stormdrifter II by University of Osnabruck
 * Based on the EURUS Balloon Code by James Coxon (GPS), Anthony Stirk's NTX2 RTTY code and Tom Igoe's SD Library.
 * Version 1.2.6 of August 18th, 2014.
 */

/* The old program broke when trying to increment the counter at the beginning of the string any time the programmer was unplugged.
 * Program probably was overloaded because it contained the code from Stormdrifter I that flew on March 5th, 2013 and the code for reading the Geiger Counter via Serial1.
 * Let's start the program back from the roots and try to integrate it step-by-step.
 */

/*
 * Read the temperature off the DS18B20 and Display on UART1 and send via RTTY (adding CRC16 Checksum). This time using the library by Miles Burton from http://www.milesburton.com/Dallas_Temperature_Control_Library .
 */

//***************LIBRARIES.************************//
#include <stdlib.h> //Used for C operators.
#include <string.h> //For using the stringf command.
#include <util/crc16.h> //Library for CRC16 generation.
#include <OneWire.h> //For OneWire Protocol.
#include <DallasTemperature.h> //Miles Burton's Temperature Sensor Library.

//******************CONSTANTS.*******************//
//First the status LEDs.
#define LED1 1
//#define LED2 22  That LED is not required.
//Define the Radio Control Pin
#define RADIOPIN 0

//*****************VARIABLES.**********************//
int linennumber = 0; 
float celsius = 0; //Temperature from the DS18B20.
//Define Buffers.
char Datastring[50]; //Full Datastring.

//************Temperature Sensor System.********************//
//The DS18B20 is on Pin 2, so let's tell the library about that.
#define ONE_WIRE_BUS 2
//Initiate OneWire Instance.
OneWire oneWire(ONE_WIRE_BUS);
//Pass Reference to the Dallas Temperature Library.
DallasTemperature sensors(&oneWire);

//*********************Checksum Generator.****************//
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
} 


//*********************SETUP ROUTINE.**********************//
void setup()
{
  pinMode(RADIOPIN, OUTPUT);
  Serial1.begin(9600); //Talk to the PC on UART1 @ 9600 bauds.
  sensors.begin(); //Start DS18B20 communications.
}
//*********************MAIN LOOP.*************************//
void loop()
{
  sensors.requestTemperatures(); //Call the function to request T from DS18B20.
  celsius = sensors.getTempCByIndex(0);
  unsigned int TempInt = celsius * 100; //Transform the float variable into an integer.

  sprintf(Datastring, "$$$$$OERNEN-II,%i,%u",linennumber,TempInt); //Build the Datastring.
  unsigned int CHECKSUM = gps_CRC16_checksum(Datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(Datastring,checksum_str);

  Serial1.println(Datastring); //Send the Datastring on UART1.
  rtty_txstring(Datastring); //Send the Datastring via the Radio.
  linennumber++; //Increase the Line Count by 1.
}

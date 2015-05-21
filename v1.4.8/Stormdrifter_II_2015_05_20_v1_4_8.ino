/*
 * Program Code for Stormdrifter II by University of Osnabruck
 * Based on:
 * EURUS Balloon Code by James Coxon (GPS)
 * Anthony Stirk's NTX2 RTTY code
 * Tom Igoe's SD Library
 * The Habduino code by Anthony Stirk.
 * The DS18B20 library of Miles Burton.
 * The SFE_BMP180 library of Sparkfun.com by Mike Grusin.
 * The Arduino Servo library.
 * Version 1.4.8 of May 20th, 2015.
 */

/* The old program broke when trying to increment the counter at the beginning of the string any time the programmer was unplugged.
 * Program probably was overloaded because it contained the code from Stormdrifter I that flew on March 5th, 2013 and the code for reading the Geiger Counter via Serial1.
 * Let's start the program back from the roots and try to integrate it step-by-step (Geiger Counter omitted).
 */

/*
 * Advanced to 1.4.8, because the Radio is now driven by an interrupt controlled by Timer2.
 * Functional Description:
 * Read the data from the GPS, check if the cutdown condition is met, read the temperature off the *two* DS18B20, 
 * the temperature and pressure from the BMP180, the temperature and humidity from the two HIH-6121,
 * read the battery voltage, as well as the two analogue Humidity Sensors (HIH-5031),
 * and the photodiode radiometer, display on UART1 and send via RTTY (adding CRC16 Checksum). Then save to SD.
 * (The analog readings are averaged over five readings)
 */

//***************LIBRARIES.************************//
#include <avr/io.h> //Used for AVR-C.
#include <avr/interrupt.h> //Used for Interrupt Control.
#include <stdlib.h> //Used for C operators.
#include <string.h> //For using the stringf command.
#include <util/crc16.h> //Library for CRC16 generation.
#include <SFE_BMP180.h> //Sparkfun BMP180 control library.
#include <Wire.h> //Two Wire Interface library (I2C).
#include <OneWire.h> //For OneWire Protocol.
#include <DallasTemperature.h> //Miles Burton's Temperature sensor library.
#include <SD.h> //SD Card library.
#include <Servo.h> //Servo library.


//+++++++++++++++Methods*************************//
void init_IO_pins(void);
void init_UART(void);
void init_BMP180(void);
void save_data(void);
void gather_send_data(void);
void getposition_gps (void);
void init_temperature_sensor(void);
void check_termination(void);
void get_pressure(void);
void get_voltage(void);
void get_analogHumidity(void);
void get_HumidIcon(void);
uint16_t gps_CRC16_checksum (char *string);

//******************CONSTANTS.*******************//
//First the status LEDs.
#define LED1 1
#define LED2 22
//Define the Radio Control Pin and RTTY Data.
#define RADIOPIN 0
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 1       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50    // Baud rate usually 50.
//Define the SPI Chip Select Pin for the SD Card.
#define chipSelect 4
//Define the control instance for the Servo Cutdown.
Servo CutServo;
//Navmode 6 -> up to 50 kilometer altitude.
#define NAVMODE_HIGH_ALTITUDE 6

//*****************VARIABLES.**********************//
int linennumber = 0; 
float celsius0 = 0, celsius1 = 0; //Temperature from the DS18B20.
//Define Buffers.
char Datastring[200]; //Full Datastring.
uint8_t buf[70]; // GPS receive buffer.
char checksum_str[6]; //CRC16 Checksum buffer.
char txstring[200];
//GPS Variables.
int32_t lat = 0, lon = 0, alt = 0, lat_dec = 0, lon_dec = 0;
uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;
unsigned long startGPS = 0;
int GPSerrorM = 0, GPSerrorL = 0, GPSerrorP = 0, GPSerrorT = 0, count = 0, n, gpsstatus, lockcount = 0;
byte navmode = 99;
byte error = 0;
int lon_int = 0, lat_int = 0;
//RTTY Variables
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
//BMP180 Variables
char status;
double T,P;
int IntTemp = 0;
long LongPress = 0;
//DS18B20 Variables
unsigned int TempInt0, TempInt1, CHECKSUM;
//SD card file designator
File dataFile;
//Cutdown Indicator
boolean cutdown = false;
char *CutdownString = "CD"; // Cutdown Message.
//Analog Variables
int intVolt = 0;
int BVolt = 0;
int32_t battvsmooth[5];
int intHum0 = 0;
int intHum1 = 0;
int anaHum0 = 0;
int anaHum1 = 0;
int32_t hum0vsmooth[5];
int32_t hum1vsmooth[5];
int intRad = 0;
int RadVolt = 0;
int32_t radvsmooth[5];

//************Temperature Sensor System.********************//
//The DS18B20 is on Pin 2, so let's tell the library about that.
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9
//Initiate OneWire Instance.
OneWire oneWire(ONE_WIRE_BUS);
//Pass Reference to the Dallas Temperature Library.
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress Thermometer0, Thermometer1;

//************Pressure Sensor System.********************//
//Create an instance of the BMP180 pressure sensor.
SFE_BMP180 pressure;


//*********************SETUP ROUTINE.**********************//
void setup()
{ 
  init_IO_pins();
  init_UART();
  init_BMP180();
  setupGPS();
  delay(100);
  init_temperature_sensor;
  initialise_interrupt();
  if (!SD.begin(chipSelect)) return;

  Serial1.println("Startup Complete.");
}


//*********************MAIN LOOP.*************************//
void loop()
{
  getposition_gps();
  
  check_termination();
  
  get_pressure();

  get_temperature();
  
//  get_HumidIcon();
  
  get_voltage();

  get_analogHumidity();

  gather_send_data();

  save_data();

  linennumber++; //Increase the Line Count by 1.
}


/* 
 * I/O Pin Initialization.
 * HARDWARE:
 * VARIABLE:
 */
void init_IO_pins(void)
{
  pinMode(RADIOPIN, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  CutServo.attach(12);
}


/* 
 * UART Initialization.
 * HARDWARE:
 * VARIABLE:
 */
void init_UART (void)
{
  Serial.begin(9600); //Establish connection to GPS.
  Serial.flush();
  Serial1.begin(9600); //Talk to the PC on UART1 @ 9600 bauds.
}

/* 
 * Interrupt Initialization.
 * HARDWARE:
 * VARIABLE:
 */
void initialise_interrupt() 
{
  // initialize Timer2
  cli();          // disable global interrupts
  TCCR2A = 0;     // set entire TCCR1A register to 0
  TCCR2B = 0;     // same for TCCR1B
  OCR2A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR2B |= (1 << WGM21);   // turn on CTC mode:
  // Set CS20, CS21 and CS22 bits for:
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS22);
  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE2A);
  sei();          // enable global interrupts
}

/* 
 * NTX2B Interrupt Routine.
 * HARDWARE: NTX2B
 * VARIABLE:
 */
ISR(TIMER2_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission. 
    strcpy(txstring,Datastring);
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else 
    {
      txstatus=0; // Should be finished
      txj=0;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);	
      txc = txc >> 1;
      break;
    }
    else 
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }

  }
}

/* 
 * NTX2B Driving Routine.
 * HARDWARE: NTX2B
 * VARIABLE:
 */
void rtty_txbit (int bit)
{
  if (bit)
  {
    digitalWrite(RADIOPIN, HIGH); // High
    digitalWrite(LED1, HIGH);
  }
  else
  {
    digitalWrite(RADIOPIN, LOW); // Low
    digitalWrite(LED1, LOW);
  }
}

/* 
 * BMP180 Initialization.
 * HARDWARE: BMP180
 * VARIABLE:
 */
void init_BMP180(void)
{
	if (pressure.begin())
    Serial1.println("BMP180 init success");
  else
  {
    Serial1.println("BMP180 init fail\n\n");
 //   while(1); // Pause forever.
  }
}

/*
 * Start DS18B20 communications. It's the Temperature sensor.
 * HARDWARE: DS18B20
 * VARIABLE:
 */
void init_temperature_sensor (void)
{
  sensors.begin();
  if (!sensors.getAddress(Thermometer0, 0)) return; 
  if (!sensors.getAddress(Thermometer1, 1)) return;
  sensors.setResolution(Thermometer0, TEMPERATURE_PRECISION);
  sensors.setResolution(Thermometer1, TEMPERATURE_PRECISION);
}


/* 
 * Setting up the GPS and collecting infos from it
 * HARDWARE: GPS (uBlox MAX-7Q)
 * VARIABLE: navmode, setNav[], error
 */
void getposition_gps (void)
{
  gps_check_nav();
  if(navmode != NAVMODE_HIGH_ALTITUDE) {
    Serial.flush();
    // Check and set the navigation mode (Airborne, 1G).
    uint8_t setNav[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC            };
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  }
  gps_check_lock(); // Check if the GPS has Lock.
  gps_get_position(); // Read the Latitude, Longitude and Altitude.
  gps_get_time(); // Read the current Time.
  error = GPSerrorM + GPSerrorL + GPSerrorP + GPSerrorT; // Add up the errors.
  print_latitude(); // Convert the Latitude into the proper format.
  print_longitude(); // Convert the Longitude into the proper format.
  if(lock == 3) digitalWrite(LED2, LOW);
  else digitalWrite(LED2, HIGH);
}

/* 
 * Get Pressure
 * HARDWARE: BMP180
 * VARIABLE: T,P,p0,status,IntTemp
 */
void get_pressure(void)
{
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      IntTemp = T * 100;
            
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
         LongPress = P * 100;
        }
   
      }

    }

  }
Serial1.println(LongPress);
}


/* 
 * Get Temperature
 * HARDWARE: DS18B20
 * VARIABLE: celsius0, celsius1, TempInt0, TempInt1
 */
void get_temperature(void)
{
  sensors.requestTemperatures(); //Call the function to request T from DS18B20.
  celsius0 = sensors.getTempCByIndex(0);
  TempInt0 = celsius0 * 100; //Transform the float variable into an integer.
  celsius1 = sensors.getTempCByIndex(1);
  TempInt1 = celsius1 * 100; //Transform the float variable into an integer.
  Serial1.println(celsius0);
  Serial1.println(celsius1);
}

/* 
 * Get Temperature and Humidity
 * HARDWARE: HIH-6121 
 * VARIABLE: celsius0, celsius1, TempInt0, TempInt1
 */
void read_HumidIcon(void)
{
	
}

/* 
 * Get Battery Voltage
 * HARDWARE: Voltage Divider
 * VARIABLE: intVolt, BVolt
 */
void get_voltage(void)
{
  intVolt = analogRead(6);
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = intVolt;
  BVolt = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;
}

/* 
 * Get Humidity from the two HIH-5031
 * HARDWARE: HIH-5031_0, HIH-5031_1
 * VARIABLE: intHum0, intHum1, anaHum0, anaHum1
 */
void get_analogHumidity(void)
{
  intHum0 = analogRead(7);
  intHum1 = analogRead(5);

  hum0vsmooth[4] = hum0vsmooth[3];
  hum0vsmooth[3] = hum0vsmooth[2];
  hum0vsmooth[2] = hum0vsmooth[1];
  hum0vsmooth[1] = hum0vsmooth[0];
  hum0vsmooth[0] = intHum0;
  anaHum0 = (hum0vsmooth[0]+hum0vsmooth[1]+ hum0vsmooth[2]+hum0vsmooth[3]+hum0vsmooth[4])/5;

  hum1vsmooth[4] = hum1vsmooth[3];
  hum1vsmooth[3] = hum1vsmooth[2];
  hum1vsmooth[2] = hum1vsmooth[1];
  hum1vsmooth[1] = hum1vsmooth[0];
  hum1vsmooth[0] = intHum1;
  anaHum1 = (hum1vsmooth[0]+hum1vsmooth[1]+ hum1vsmooth[2]+hum1vsmooth[3]+hum1vsmooth[4])/5;
}

/* 
 * Get Light Intensity from Radiometer
 * HARDWARE: PD Radiometer
 * VARIABLE: intRad, RadVolt
 */
void get_Radiometer(void)
{
  intRad = analogRead(4);

  radvsmooth[4] = radvsmooth[3];
  radvsmooth[3] = radvsmooth[2];
  radvsmooth[2] = radvsmooth[1];
  radvsmooth[1] = radvsmooth[0];
  radvsmooth[0] = intRad;
  RadVolt = (radvsmooth[0]+radvsmooth[1]+ radvsmooth[2]+radvsmooth[3]+radvsmooth[4])/5;
}

/* 
 * Link all data to string and send it over UART1 and Radio
 * HARDWARE: GPS, RADIO
 * VARIABLE: linennumber, hour, minute, second, lat, lat_int, lat_dec, lon,
 *           lon_int,lon_dec, alt, navmode, error, lock, sats, TempInt
 */
void gather_send_data(void)
{
  sprintf(Datastring, "$$$$$OERNEN-II,%i,%02d:%02d:%02d,%s%i.%07ld,%s%i.%07ld,%ld,%d,%d,%d,%d,%u,%lu,%u,%u,%u,%u,%u", 
  linennumber, hour, minute, second, lat < 0 ? "-" : "",lat_int,lat_dec, lon < 0 ? "-" : "",
  lon_int,lon_dec, alt, navmode, error, lock, sats, IntTemp, LongPress, TempInt0, TempInt1, anaHum0, anaHum1, BVolt); //Build the Datastring.
  CHECKSUM = gps_CRC16_checksum(Datastring);  // Calculates the checksum for this datastring
  
   if(cutdown == true)
  {
   strcat(Datastring, CutdownString); 
  }

  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(Datastring,checksum_str);

  Serial1.println(Datastring); //Send the Datastring on UART1.
//  rtty_txstring(Datastring); //Send the Datastring via the Radio.
}


/* 
 * Save data to SD-card
 * HARDWARE: SD-card
 * VARIABLE: dataFile,  
 */
void save_data(void)
{
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(Datastring);
    dataFile.close();
  }  
}

/* 
 * Check if Cutdown/Termination Condition is met.
 * HARDWARE: Servo Controller
 * VARIABLE:
 */
void check_termination(void)
{
 if(lock != 3 && sats == 0 || alt < 500.0)
  {
    CutServo.write(0);
  }
  else if(alt > 30480.0) // Is the Altitude greater than X m? Then Activate the Cutdown.
  {
  CutServo.write(180);
  cutdown = true;
  }
  else if(alt < 30480.0)
  CutServo.write(0); 
}

/* 
 * Checksum Generator
 * generates CRC16 Checksum
 * HARDWARE:
 * VARIABLE:
 */
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first five $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
} 


/**
  * This is the RTTY driven firmware for the Pecan Pico 4 which is developed
  * by Thomas Krahn. The initial Software will transmit one GPS position every
  * three to five minutes with 10mW.
  * The transmitter chip beeing used, is a Si4464, which is able to cover a
  * frequency range from 119 MHz until 1050 MHz at 100mW max.
  * --------------------------------------------------------------------------------
  * Loop Actions
  * 
  * After the software started, it will remain in a loop doing folloing steps:
  *     - Transmitting 30 single RTTY Beeps every 5 seconds        150 seconds
  *     - Switch on GPS
  *     - Transmitting Double RTTY Beeps every 5 seconds until    ~ 15 seconds
  *       GPS locks but 30 Beeps max.
  *     - Switch off
  *     - Triple RTTY Beep one time                                  5 seconds
  *     - Transmitting one packet of acquired data                  10 seconds
  *                                                              = 180 seconds for one cycle
  * --------------------------------------------------------------------------------
  * Transmission Sentence
  * 
  * The software will transmit following sentence. Example for D-1:
  * $$D-1,16,18:22:48,52.31930,13.64217,583,4,13,100041,1.104*AD28
  * 
  *               Format      Value
  * Callsign      String      D-1
  * Count         Integer     16
  * Time          00:00:00    18:22:48
  * Latitude      Integer     52.31930
  * Longitude     Integer     13.64217
  * Altitude      Integer     583
  * Satellites    Integer     4
  * Temperature   Integer     13
  * Pressure      Integer     100041
  * Voltage       Integer     1.104
  * CRC           -           AD28
  * --------------------------------------------------------------------------------
  * Known issues
  * 
  * The Pecan has an error depending on its design. Sometimes it happens, Serial
  * transmissions between GPS and ATMega will not work. The ATMega will get broken
  * data indicated by the CRC Check. In this case, the tracker retries to get
  * the Position or Time data up to 10 times. In order to get the data again,
  * Double Beeps sometimes have a delay.
  * 
  * MAX6 and MAX7 have a random hopping caused by switching it off in the sleep
  * intervals. This can be avoided by let it switched on for the whole time, which
  * is not recommended, because its the part on the PCB consuming the most power.
  * --------------------------------------------------------------------------------
  * @file PecanAva.ino
  * @version 2.0
  * @author Sven Steudte
  * 
  * Some other authors created parts of the code before
  * 
  * @author Anthony Stirk   Interrupt method
  * @author Jon Sowman      GPS Decoding
  * @author Thomas Krahn    First version
  */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <Wire.h>
#include "sensors.h"
#include "Si446x.h"

//Tracker Configuration
#define CALLSIGN "D-1"                  //Callsign
#define ASCII 7                         //ASCII 7 or 8
#define STOPBITS 2                      //Either 1 or 2
#define TXDELAY 25                      //Transmit-Delay in bit
#define RTTY_BAUD 50                    //Baud rate, max = 600
#define RADIO_FREQUENCY 145.300         //Transmit frequency in MHz
#define RTTY_SHIFT 440                  //RTTY shift in Hz (varies, differs also on 2m and 70cm)
                                        //490 = 450 Hz @ 434.500 Mhz
                                        //440 = 425 Hz @ 145.300 Mhz


//PCB Configuration
#define RADIO_PIN 10                    //CS pin that defines the SPI slave
#define RADIO_SDN RADIO_SDN_PIN         //Pin to power off the transmitter

#define STATUS_LED 13                   //Status LED (Green)
#define GPS_POWER_PIN 5                 //On Pecan boards the uBlox GPS has its own regulator that needs to get enabled.
#define GPS_RESET_PIN 9                 //Pull this pin to LOW to reset the GPS

#define RADIO_POWER 20                  //6    0dBm  (1mW)
                                        //16   8dBm  (6mW)
                                        //20  10dBm  (10mW)
                                        //32  14dBm  (25mW)
                                        //40  17dBm  (50mW)
                                        //127 20dBm  (100mW max)

//Global Variables
Si446x radio1(RADIO_PIN);               //Radio object
uint8_t buf[60];                        //GPS String buffer
char txstring[100];                     //Transmitting buffer
volatile int txstringlength =  0;       //Transmitting buffer length
volatile int txstatus = 0;              //Current TX state

volatile char txc;                      //Current Char to be transmitted
volatile int txi;                       //
volatile int txj;                       //
volatile long count = 1;                //Incremental number of packets transmitted

uint8_t lock = 0;                       //GPS lock
                                        //0 = Invalid lock
                                        //3 = Valid lock

int navmode = 0;                        //
int battv = 0;                          //
int GPSerror = 0;                       //GPS error code

uint8_t hour = 0;                       //Hour of day
uint8_t minute = 0;                     //Minute of hour
uint8_t second = 0;                     //Second of minute
int32_t lat = 0;                        //Latitude
int32_t lon = 0;                        //Longitude
int lat_int = 0;                        //Digit before decimal point of Latitude
int32_t lat_dec = 0;                    //Decimal digits of Latitude
int lon_int = 0;                        //Digit before decimal point of Longitude
int32_t lon_dec = 0;                    //Decimal digits of Longitude
int32_t alt = 0;                        //Altitude
uint8_t sats = 0;                       //Active satellites used
short bmp085temp;                       //Internal Temperature of BMP085 (Pressure sensor)
long bmp085pressure;                    //Air pressure
long bat_mv;                            //Battery voltage in millivolts

/**
  * Setup function of the program. Initializes hardware components.
  * Will be called once at the beginning.
  */
void setup() {
  //Set Pin mode
  pinMode(STATUS_LED, OUTPUT);         //Status LED (Green)
  pinMode(GPS_POWER_PIN, OUTPUT);      //GPS Power
  pinMode(GPS_RESET_PIN, OUTPUT);      //GPS Reset
  pinMode(RADIO_SDN, OUTPUT);          //Radio
  
  Serial.begin(9600);                  //Start Serial
  
  digitalWrite(GPS_POWER_PIN, LOW);    //Power off GPS
  digitalWrite(GPS_RESET_PIN, HIGH);   //Disable Reset pin of GPS
    
  digitalWrite(RADIO_SDN, HIGH);       //Power on Radio
  setupRadio();                        //Setup radio
  
  sensors_setup();                     //Setup sensors
  
  initialise_interrupt();              //Initialize interrupt
}

/**
  * Function which is called by the microcontroller first time, when setup is completed and
  * when this function is finished.
  */
void loop() {
  //Single beep on radio
  for(int i=0; i<30; i++) {
    radio1.ptt_on();
    radio1.setHighTone();
    delay(20);
    radio1.setLowTone();
    delay(80);
    radio1.stop_tx();
    delay(3600);
  }
  
  //Switch on GPS
  digitalWrite(GPS_POWER_PIN, HIGH);
  delay(500);
  setupGPS(); //Setup GPS
  
  //Double beep on radio until aquired GPS lock with 4 satellites (Loop max. 30 times)
  int gpsLoops = 0;
  do {
    //Beep
    radio1.ptt_on();
    radio1.setHighTone();
    delay(20);
    radio1.setLowTone();
    delay(80);
    radio1.setHighTone();
    delay(80);
    radio1.setLowTone();
    delay(80);
    radio1.stop_tx();
    delay(2300);
    
    //Request data from GPS
    prepare_data();
  } while(sats < 4 && lock != 3 && gpsLoops++ < 30);
  
  //Switch off GPS
  digitalWrite(GPS_POWER_PIN, LOW);
  
  //Triple beep on radio
  radio1.ptt_on();
  radio1.setHighTone();
  delay(20);
  radio1.setLowTone();
  delay(80);
  radio1.setHighTone();
  delay(80);
  radio1.setLowTone();
  delay(80);
  radio1.setHighTone();
  delay(80);
  radio1.setLowTone();
  delay(80);
  radio1.stop_tx();
  delay(2200);
  
  //Transmitting data by interrupt function
  radio1.ptt_on();
  txstatus = 3;
  
  //Wait until data is sent by interrupt function
  while(txstatus != 0)
    delay(500);
}

/**
  * Turns off all GPS NMEA strings apart on the uBlox module.
  */
void setupGPS() {
  uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
    0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25,
    0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xA0, 0xA9
  };
  sendUBX(setNMEAoff, sizeof(setNMEAoff) / sizeof(uint8_t));
  delay(500);
  setGPS_DynamicModel6();
}
/**
  * Transmits a message to the UBlox module
  * @param MSG Message
  * @param len Length of the message
  */
void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  delay(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

uint8_t gps_check_nav(void) {
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84
  };
  sendUBX(request, 8);

  //Receive message from the GPS
  gps_get_data();

  //Verify sync and header bytes
  if(buf[0] != 0xB5 || buf[1] != 0x62) {
    GPSerror = 41;
  }
  if(buf[2] != 0x06 || buf[3] != 0x24) {
    GPSerror = 42;
  }
  
  //Check message checksum
  if(!_gps_verify_checksum(&buf[2], 40)) {
    GPSerror = 43;
  }
  
  //Return navigation mode and let the caller analyse it
  navmode = buf[8];
}

void gps_get_data() {
  Serial.flush();
  
  //Clearing buffer
  for(int i=0; i<60; i++) {
    buf[i] = 0;
  }
  
  unsigned long startTime = millis();
  int i = 0;
  
  while((i < 60) && ((millis() - startTime) < 1000)) {
    if(Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}

bool _gps_verify_checksum(uint8_t* data, uint8_t len) {
  uint8_t a,b;
  gps_ubx_checksum(data, len, &a, &b);
  return !( a != *(data + len) || b != *(data + len + 1));
}

void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb) {
  *cka = 0;
  *ckb = 0;
  for(uint8_t i=0; i<len; i++) {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  //Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	  //Header
  ackPacket[1] = 0x62;	  //Header
  ackPacket[2] = 0x05;	  //Class
  ackPacket[3] = 0x01;	  //ID
  ackPacket[4] = 0x02;    //Length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  //ACK class
  ackPacket[7] = MSG[3];  //ACK ID
  ackPacket[8] = 0;       //CK_A
  ackPacket[9] = 0;       //CK_B

  //Calculate checksums
  for(uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while(1) {
    // Test for success
    if(ackByteID > 9) { //All packets in order
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if(millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if(Serial.available()) {
      b = Serial.read();

      //Check that bytes arrive in sequence as per expected ACK packet
      if(b == ackPacket[ackByteID]) {
        ackByteID++;
      } else {
        ackByteID = 0; //Reset and look again, invalid order
      }
    }
  }
}

void gps_check_lock() {
  GPSerror = 0;
  Serial.flush();
  
  //Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00, 0x07, 0x16
  };
  sendUBX(request, 8);

  //Get the message back from the GPS
  gps_get_data();
  
  //Verify the sync and header bits
  if(buf[0] != 0xB5 || buf[1] != 0x62) {
    GPSerror = 11;
  }
  if(buf[2] != 0x01 || buf[3] != 0x06) {
    GPSerror = 12;
  }

  //Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if(!_gps_verify_checksum(&buf[2], 56)) {
    GPSerror = 13;
  }

  if(GPSerror == 0) {
    //Return the value if GPSfixOK is set in 'flags'
    lock = buf[17] & 0x01 ? buf[16] : 0;
    sats = buf[53];
  } else {
    lock = 0;
  }
}

void setGPS_DynamicModel6() {
  int gps_set_sucess = 0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
    0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x16, 0xDC
  };
  while(!gps_set_sucess) {
    sendUBX(setdm6, sizeof(setdm6) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setdm6);
  }
}

void setGPS_DynamicModel3() {
  int gps_set_sucess = 0;
  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
    0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x13, 0x76
  };
  while(!gps_set_sucess) {
    sendUBX(setdm3, sizeof(setdm3) / sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm3);
  }
}

void gps_get_position() {
  GPSerror = 0;
  Serial.flush();
  
  //Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A
  };
  sendUBX(request, 8);

  //Receive message from the GPS
  gps_get_data();

  //Verify the sync and header bits
  if(buf[0] != 0xB5 || buf[1] != 0x62)
    GPSerror = 21;
  if(buf[2] != 0x01 || buf[3] != 0x02)
    GPSerror = 22;

  if(!_gps_verify_checksum(&buf[2], 32)) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    //4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;

    lon_int = abs(lon / 10000000);
    lon_dec = (labs(lon) % 10000000) / 100;
    
    //4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;

    lat_int = abs(lat / 10000000);
    lat_dec = (labs(lat) % 10000000) / 100;

    //4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000; //Convert to meter
  }
}

void gps_get_time() {
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67
  };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(hour > 23 || minute > 59 || second > 59) {
      GPSerror = 34;
    } else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}

uint16_t gps_CRC16_checksum(char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two chars ($$)
  for(i = 5; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

ISR(TIMER1_COMPA_vect) {
  switch(txstatus) {
    case 3: //Forming transmission string
      sprintf(txstring, "$$$$$%s,%ld,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d", CALLSIGN, count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, alt,sats);
      sprintf(txstring, "%s,%d,%ld,%ld.%03ld", txstring, bmp085temp, bmp085pressure, bat_mv / 1000l, bat_mv % 1000l);
      sprintf(txstring, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
      
      txstringlength = strlen(txstring);
      txstatus = 6;
      txj = 0;
      
      txstatus = 6;
      break;
      
    case 6: //TX-delay
      txj++;
      if(txj > TXDELAY) { 
        txj = 0;
        txstatus = 7;
      }
      break;
    
    case 7: //Transmit a single char
      if(txj < txstringlength) {
        txc = txstring[txj]; //Select char
        txj++;
        txstatus = 8;
        rtty_txbit(0); //Start Bit (Synchronizing)
        txi = 0;
      } else {
        txstatus = 0; //Finished to transmit char
        txj = 0;
        count++;
      }
      break;
    
    case 8:
      if(txi < ASCII) {
        txi++;
        if(txc & 1) {
          rtty_txbit(1);
        } else {
          rtty_txbit(0);
        }
        txc = txc >> 1;
      } else {
        rtty_txbit(1); //Stop Bit
        txstatus = 9;
        txi = 0;
      }
      break;
    
    case 9:
      if(STOPBITS == 2)
        rtty_txbit(1); //Stop Bit
      txstatus = 7;
  }
}

void rtty_txbit(int bit) {
  if(bit) {
    radio1.setHighTone();
  } else {
    radio1.setLowTone();
  }
}

void setupRadio() {
  radio1.initSPI();
  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.setShift(RTTY_SHIFT);
  radio1.setPowerLevel(RADIO_POWER);
  radio1.init();
}

void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5
  };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}
void prepare_data() {
  int i = 0;
  do {
    gps_check_lock();
  } while(GPSerror != 0 && i++ < 10);
  
  i = 0;
  do {
    gps_get_position();
  } while(GPSerror != 0 && i++ < 10);
  
  i = 0;
  do {
    gps_get_time();
  } while(GPSerror != 0 && i++ < 10);

  bmp085temp = bmp085GetTemperature(bmp085ReadUT()) / 10; // Must read temperature from BMP085 before you can read pressure!
  bmp085pressure = bmp085GetPressure(bmp085ReadUP());
  bat_mv = getUBatt();
  
}


void initialise_interrupt() {
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void blinkled(int blinks) {
  for(int i = 0; i <= blinks; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(500);
    digitalWrite(STATUS_LED, LOW);
    delay(500);
  }    
}

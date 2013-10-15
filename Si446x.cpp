// Si446x lib by KT5TK
// based on (and hopefully somewhat compatible to)
// RFM22 Lib - edited by James Coxon jacoxon@googlemail.com 2011

#include "Si446x.h"

unsigned int si446x_powerlevel = 0;
unsigned long active_freq = DEFAULT_FREQ;
unsigned long active_shift = DEFAULT_SHIFT;
unsigned int active_level = DEFAULT_POWER_LEVEL;

int outdiv = 4;


uint8_t Si446x::read(uint8_t addr) const {
	//write ss low to start
	digitalWrite(pin, LOW);
	
	// make sure the msb is 0 so we do a read, not a write
	addr &= 0x7F;
	SPI.transfer(addr);
	uint8_t val = SPI.transfer(0x00);
	
	//write ss high to end
	digitalWrite(pin, HIGH);
	
	return val;
}

void Si446x::write(uint8_t addr, uint8_t data) const {
	//write ss low to start
	digitalWrite(pin, LOW);
	
	// make sure the msb is 1 so we do a write
	addr |= 0x80;
	SPI.transfer(addr);
	SPI.transfer(data);
	
	//write ss high to end
	digitalWrite(pin, HIGH);
}

void Si446x::read(uint8_t start_addr, uint8_t buf[], uint8_t len) {
	//write ss low to start
	digitalWrite(pin, LOW);

	// make sure the msb is 0 so we do a read, not a write
	start_addr &= 0x7F;
	SPI.transfer(start_addr);
	for (int i = 0; i < len; i++) {
		buf[i] = SPI.transfer(0x00);
	}

	//write ss high to end
	digitalWrite(pin, HIGH);
}
void Si446x::write(uint8_t start_addr, uint8_t data[], uint8_t len) {
	//write ss low to start
	digitalWrite(pin, LOW);

	// make sure the msb is 1 so we do a write
	start_addr |= 0x80;
	SPI.transfer(start_addr);
	for (int i = 0; i < len; i++) {
		SPI.transfer(data[i]);
	}

	//write ss high to end
	digitalWrite(pin, HIGH);
}


void Si446x::SendCmdReceiveAnswer(int byteCountTx, int byteCountRx, const char* pData)
{
    
    /* There was a bug in A1 hardware that will not handle 1 byte commands. 
       It was supposedly fixed in B0 but the fix didn't make it at the last minute, so here we go again */
    if (byteCountTx == 1)
        byteCountTx++;
    
    digitalWrite(SSpin,LOW);
    char answer;   
      
    for (int j = 0; j < byteCountTx; j++) // Loop through the bytes of the pData
    {
      byte wordb = pData[j];
      SPI.transfer(wordb);  
    } 
    
    digitalWrite(SSpin,HIGH);

    delayMicroseconds(20);

    digitalWrite(SSpin,LOW);   
    
    int reply = 0x00;
    while (reply != 0xFF)
    {       
       reply = SPI.transfer(0x44);
       //Serial.print(reply,HEX);
       //Serial.print(" ");
       if (reply != 0xFF)
       {
         digitalWrite(SSpin,HIGH);
         delayMicroseconds(20);
         digitalWrite(SSpin,LOW);   
       }
    }
    
//    Serial.println();
    
//    Serial.print("rx: ");
    
   
//    for (int k = 1; k < byteCountRx; k++)
//    {
//      Serial.print(SPI.transfer(0x44), HEX);
//      Serial.print(" ");
//    }
       
    digitalWrite(SSpin,HIGH);
//    Serial.println();
//    delay(500); // Wait half a second to prevent Serial buffer overflow
    delay(50);   //make sure spi communication has finished
}



void Si446x::resetFIFO() {
//	write(0x08, 0x03);
//	write(0x08, 0x00);
// not implemented for Si446x
}


void Si446x::sendFrequencyToSi446x(unsigned long freq)
{
  // Set the output divider according to recommended ranges given in Si446x datasheet  
  int band = 0;
  if (freq < 705000000UL) { outdiv = 6;  band = 1;};
  if (freq < 525000000UL) { outdiv = 8;  band = 2;};
  if (freq < 353000000UL) { outdiv = 12; band = 3;};
  if (freq < 239000000UL) { outdiv = 16; band = 4;};
  if (freq < 177000000UL) { outdiv = 24; band = 5;};
  
 
  unsigned long f_pfd = 2 * VCXO_FREQ / outdiv;
  
  unsigned int n = ((unsigned int)(freq / f_pfd)) - 1;
  
  float ratio = (float)freq / (float)f_pfd;
  float rest  = ratio - (float)n;
  

  unsigned long m = (unsigned long)(rest * 524288UL); 
 


// set the band parameter
  unsigned int sy_sel = 8; // 
  char set_band_property_command[] = {0x11, 0x20, 0x01, 0x51, (band + sy_sel)}; //   
  // send parameters
  SendCmdReceiveAnswer(5, 1, set_band_property_command);

// Set the pll parameters
  unsigned int m2 = m / 0x10000;
  unsigned int m1 = (m - m2 * 0x10000) / 0x100;
  unsigned int m0 = (m - m2 * 0x10000 - m1 * 0x100); 
  
  unsigned long channel_increment = 524288 * outdiv * active_shift / (2 * VCXO_FREQ);
  char c1 = channel_increment / 0x100;
  char c0 = channel_increment - (0x100 * c1);
  
  // assemble parameter string
  char set_frequency_property_command[] = {0x11, 0x40, 0x06, 0x00, n, m2, m1, m0, c1, c0};
  // send parameters
  SendCmdReceiveAnswer(10, 1, set_frequency_property_command);
  
  // Set the Power
  char set_pa_pwr_lvl_property_command[] = {0x11, 0x22, 0x01, 0x01, active_level};
  // send parameters
  SendCmdReceiveAnswer(5, 1, set_pa_pwr_lvl_property_command);
  

}



void Si446x::init() {
  
  pinMode(VCXO_ENABLE_PIN, OUTPUT);
  pinMode(GPIO0_PIN, OUTPUT);
  
  digitalWrite(VCXO_ENABLE_PIN,HIGH);
  //Serial.println("VCXO is enabled"); 
  
  
  delay(100);
  
//ToDo: put the VCXO to mid frequency 
  
  

  digitalWrite(RADIO_SDN_PIN, HIGH);  // active high shutdown = reset
  delay(600);
  digitalWrite(RADIO_SDN_PIN, LOW);   // booting
  //Serial.println("Radio is powered up"); 

  // Start talking to the Si446X radio chip

  const char PART_INFO_command[] = {0x01}; // Part Info
  SendCmdReceiveAnswer(1, 9, PART_INFO_command);
  //Serial.println("Part info was checked");

//divide VCXO_FREQ into its bytes; MSB first  
  unsigned int x3 = VCXO_FREQ / 0x1000000;
  unsigned int x2 = (VCXO_FREQ - x3 * 0x1000000) / 0x10000;
  unsigned int x1 = (VCXO_FREQ - x3 * 0x1000000 - x2 * 0x10000) / 0x100;
  unsigned int x0 = (VCXO_FREQ - x3 * 0x1000000 - x2 * 0x10000 - x1 * 0x100); 

//POWER_UP
  const char init_command[] = {0x02, 0x01, 0x01, x3, x2, x1, x0};// no patch, boot main app. img, FREQ_VCXO, return 1 byte
  SendCmdReceiveAnswer(7, 1 ,init_command); 

  //Serial.println("Radio booted"); 

  const char get_int_status_command[] = {0x20, 0x00, 0x00, 0x00}; //  Clear all pending interrupts and get the interrupt status back
  SendCmdReceiveAnswer(4, 9, get_int_status_command);


  //Serial.println("Radio ready");
 
  const char gpio_pin_cfg_command[] = {0x13, 0x04, 0x02, 0x02, 0x02, 0x08, 0x11, 0x00}; //  Set GPIO0 as input, all other GPIOs LOW; Link NIRQ to CTS; Link SDO to MISO; Max drive strength
  SendCmdReceiveAnswer(8, 8, gpio_pin_cfg_command);

  //Serial.println("LEDs should be switched off at this point");
  
  sendFrequencyToSi446x(active_freq);
  //Serial.println("Frequency set");  

 
  setModem(); 
  //Serial.println("CW mode set");  
  
  setDeviation(active_shift);
  
  tune_tx();
  //Serial.println("TX tune");  
  
  setHighTone();


}

void Si446x::initSPI() {  // Is this really needed for Si446x?
	SPI.begin();
	// Si446x seems to speak spi mode 0
	SPI.setDataMode(SPI_MODE0);
	// Setting clock speed to 8mhz, as 10 is the max for the rfm22
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	// MSB first
	//SPI.setBitOrder(MSBFIRST);
}

// Configuration parameter functions ---------------------------------------

void Si446x::setModem()
{
  // Set to CW mode
  //Serial.println("Setting modem into direct asynchronous 2FSK mode using GPIO0");  
  char set_modem_mod_type_command[] = {0x11, 0x20, 0x01, 0x00, 0b10001010};
  SendCmdReceiveAnswer(5, 1, set_modem_mod_type_command);
  
  
}


void Si446x::setDeviation(unsigned long deviation)
{
  //Make sure that Si446x::sendFrequencyToSi446x() was called before this function, so that we have set the global variable 'outdiv' properly
  //outdiv = 8;
  float units_per_hz = (( 0x40000 *  outdiv ) / (float)VCXO_FREQ);
  // Set deviation for RTTY
  unsigned long modem_freq_dev = (unsigned long)(units_per_hz * deviation / 2.0 );
  unsigned long mask = 0b11111111;
  char modem_freq_dev_0 = mask & modem_freq_dev;
  char modem_freq_dev_1 = mask & (modem_freq_dev >> 8);
  char modem_freq_dev_2 = mask & (modem_freq_dev >> 16);
  
  
  char set_modem_freq_dev_command[] = {0x11, 0x20, 0x03, 0x0A, modem_freq_dev_2, modem_freq_dev_1, modem_freq_dev_0};
  SendCmdReceiveAnswer(7, 1, set_modem_freq_dev_command);  
  
  
  
}

boolean Si446x::setPowerLevel(unsigned int level)
{
  boolean success = false;
  if (level > 1 && level < 128) {success = true;};  
  active_level = level;
  return success;

}

boolean Si446x::setFrequency(float fFreq)  // fFreq in MHz
{
  boolean success = false;
  if (fFreq > 119.0 && fFreq < 1050.0) {success = true;};  
  active_freq = (unsigned long)(fFreq * 1000000);
  return success;
}

boolean Si446x::setShift(unsigned long shift) // shift in Hz
{
  boolean success = false;
  if (shift > 1 && shift < 10000) {success = true;};  
  active_shift = shift;
  return success;
}



void Si446x::start_tx(char channel)
{
//  char change_state_command[] = {0x34, 0x07}; //  Change to TX state
//  SendCmdReceiveAnswer(2, 1, change_state_command);
  //tune_tx();
  char start_tx_command[] = {0x31, channel, 0x30, 0x00, 0x00, 0x00};
  SendCmdReceiveAnswer(6, 1, start_tx_command);
}

void Si446x::stop_tx()
{
  char change_state_command[] = {0x34, 0x03}; //  Change to Ready state
  SendCmdReceiveAnswer(2, 1, change_state_command);

}

void Si446x::tune_tx()
{
  char change_state_command[] = {0x34, 0x05}; //  Change to TX tune state
  SendCmdReceiveAnswer(2, 1, change_state_command);

}

void Si446x::ptt_on()
{
  
  digitalWrite(VCXO_ENABLE_PIN, HIGH);
  init();
  // turn on the blue LED (GPIO2) to indicate TX
  char gpio_pin_cfg_command2[] = {0x13, 0x02, 0x02, 0x03, 0x02, 0x08, 0x11, 0x00}; //  Set GPIO2 HIGH; Link NIRQ to CTS; Link SDO to MISO; Max drive strength
  SendCmdReceiveAnswer(8, 1, gpio_pin_cfg_command2);

  start_tx(1);
  si446x_powerlevel = 1023;
}

void Si446x::ptt_off()
{
  stop_tx();
  si446x_powerlevel = 0;
  // turn off the blue LED (GPIO2)
  char gpio_pin_cfg_command0[] = {0x13, 0x02, 0x02, 0x02, 0x02, 0x08, 0x11, 0x00}; //  Set all GPIOs LOW; Link NIRQ to CTS; Link SDO to MISO; Max drive strength
  SendCmdReceiveAnswer(8, 1, gpio_pin_cfg_command0);

  digitalWrite(RADIO_SDN_PIN, HIGH);  // active high = shutdown
  //digitalWrite(VCXO_ENABLE_PIN, LOW); //keep enabled for now

}


void Si446x::setHighTone()
{
  //analogWrite(VCXO_CONTROL_PIN, 131);
  digitalWrite(GPIO0_PIN, HIGH);
}

void Si446x::setLowTone()
{
  //analogWrite(VCXO_CONTROL_PIN, 124);
  digitalWrite(GPIO0_PIN, LOW);
}

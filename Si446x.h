
#ifndef Si446x_h
#define Si446x_h
#include <SPI.h>


#define VCXO_FREQ 27000000       // Frequency of the crystal (oscillator) attached to the Si446x in Hz 
#define DEFAULT_FREQ 433550000   // Frequency for the first initialisation of the transmitter in Hz
#define DEFAULT_SHIFT 400        // Shift (=channel spacing) for the first initialisation of the transmitter in Hz
#define DEFAULT_POWER_LEVEL 20   // Power level of the PA





// PIN Definitions:
#define RADIO_SDN_PIN   7        // To switch the Si446x on and off
#define VCXO_ENABLE_PIN 8        // To switch the VCXO on and off
#define VCXO_CONTROL_PIN 3       // PWM output to set the VCXO frequency
#define GPIO0_PIN 4              // Direct digital modulation pin (connected to the the GPIO0 pin on the Si446x)

#define SCKpin  13   // SCK
#define SSpin   pin  // SS
#define MOSIpin 11   // MOSI
#define MISOpin 12   // MISO


class Si446x
{
	int pin;
public:
	Si446x(uint8_t pin) : pin(pin) 
	{
		pinMode(pin, OUTPUT);
		digitalWrite(pin, HIGH);
	}
	
	uint8_t read(uint8_t addr) const;
	void write(uint8_t addr, uint8_t data) const;
	
	void read(uint8_t start_addr, uint8_t buf[], uint8_t len);
	void write(uint8_t start_addr, uint8_t data[], uint8_t len);
	void resetFIFO();
	
	boolean setFrequency(float fFreq);     // fFreq in MHz
	boolean setShift(unsigned long shift); // shift in Hz

	void init();
	
	static void initSPI();
        virtual void ptt_on();
        virtual void ptt_off();
        virtual void setHighTone();
        virtual void setLowTone();
        boolean setPowerLevel(unsigned int level);
        void start_tx(char channel);
        void stop_tx(void);


private:
        void SendCmdReceiveAnswer(int byteCountTx, int byteCountRx, const char* pData);
        void setModem(void);
        void tune_tx(void);
        void sendFrequencyToSi446x(unsigned long freq); 
        void setDeviation(unsigned long deviation) ;
};

#endif

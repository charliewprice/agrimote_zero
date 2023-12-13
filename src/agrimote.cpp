#include <Arduino.h>

/********************************************************************************************************************************************
 *
 * AgriMote (c)  2020, 2021, 2022 Charlie Price
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 *
 * To use this sketch, the application and device must be created
 * in the Chirpstack Application Server
 *
 * Many device can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * the radio type is defined in the MCCI_LoraWan_LMIC library:
 *   .../project_config/lmic_project_config.h
 *
 *   02-28-2020  Added the TPL5010 nanp-power watchdog timer to reset the MPU on lockups
 *               using a 68K resistor on the delay pin yields a 1724second (~30minute) period.
 *               SELECT
 * 					"timestamp",
 * 					"timestamp" - lag("timestamp", 1) OVER (ORDER BY "timestamp") AS delta,
 * 					(EXTRACT(epoch FROM ("timestamp" - lag("timestamp", 1) OVER (ORDER BY "timestamp"))))::int AS seconds
 * 				 FROM kanji_eventlog WHERE node_id=20001 AND "timestamp" > NOW() - INTERVAL '10 hours'
 * 				 ORDER BY "timestamp" desc;
 *   ----------
 *   03-04-2020	 Add support for I2C EEPROM used to store the DevEUI for the mote
 *   04-14-2020  Another hang without a Watchdog reset.
 *   				o- LED was ON, occurred at frame #322 on agMote_20002
 *   				o -WATCHDOG_DONE was stuck HIGH on oscilloscope;
 *   				o- On a RESET WATCHDOG_DONE went HIGH and stayed HIGH for several seconds
 *   				   (maybe in setup?) instead of a BLIP only after TX_COMPLETE as expected.
 *   			 	a- added a pinMode(WATCHDOG_DONE), and set to LOW coming out of sleep
 *   			 	a- moved pin configuration to top of setup();
 *   			 	q- could there be crosstalk with _LED pin?  agMote_20003 not having this issue.
 *   			 	a- temporarily setting _LED as INPUT
 *   04-15-2020     a- replace reference to _LED with LED_BUILTIN (defined by LMIC)
 *   				a- turn off LED_BUILTIN in TX_START event
 *   04-24-2020     o- looks like the BOD is resetting the MPU
 *   				a- configure regulator so that it doesn't go into micro-power mode on standby
 *   04-28-2020     o- still seeing hang on mote 20002
 *                  a- move tickling the watchdog to loop()
 *                  a- use averaging on fuel gauge
 *                  a- cleanup code, remove references to RocketScream, 32u4, etc.
 *   04-29-2020     a- begin work on water trough monitor sensor
 *   05-11-2020	    o- high frame counts seem to cause packet send failures
 *                  a- reset the processor when frame count reaches a threshold
 *   05-13-2020     a- add the water level sensor logic
 *   05-26-2020     a- add support for a GEOLOCATION mote type
 *   06-13-2020     o- low SNR/RSSI condition causes node to disappear without WATCHDOG reset
 *                  			a- every 5th packet will request gateway CONFIRMATION/ACK
 *   07-04-2020     a- moving BME680 from I2C to SPI interface to avoid bus lockups
 *   03-02-2021		added Current Transformers as a sensor type
 *   03-20-2021     added Mains Sensor as a sensor type
 *
 *   05-27-2022		support for reading state of switches such as end-stop, opto-interrupters
 *
 *******************************************************************************************************************************************/

#include <Wire.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <RTCZero.h>

//#define _I2C_WATCHDOG

#if defined(_I2C_WATCHDOG)
#include <Adafruit_SleepyDog.h>
#endif

#include "project_config/lmic_project_config.h"

#define _FIXED_CHANNEL 10	// 904.3MHz is Ch10 in US915 Bullwinkle & Sherman NanoGateways only!
#define _ACK_PACKETS		// #16 low SNR/RSSI

#define _DEBUG_SERIAL
//#define _USE_NEOPIXEL
//#define _TEST_NEOPIXEL
#define _DONT_SLEEP
#define _DUMP_KEYS
#define _SEND_INTERVAL_MINUTES 1

#if defined _USE_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#if defined(BOARD_HAPPYSOW)
#define PIXELPIN       5
#define NUMPIXELS      1
#else
#define PIXELPIN       11
#define NUMPIXELS      2
#endif
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);
#endif

//#define _MAX_DATA_SIZE 52
#define _BLINK_MILLIS 2000
#define _SEND_MILLIS 60000

#define _EEPROM_IC2_ADDR 	0x50		//the I2C bus address for the EEPROM
#define _DEVEUI_ADDR 		0xF8		//the location in the EEPROM for the DEVEUI

// The mote's sensor type (select ONE ONLY)
//#define _BME680				// Bosch BME680 sensor for temperature, humidity, barometric pressure, and gas resistance
//#define _BME680_CT_PROFILER     // a combination of current probes and line-voltage detector
//#define _TMP36				// diode type temperature sensor with analog output
//#define _MCP9808				// diode type temperature sensor with I2C interface
#define _18B20				    // 2-wire DalSemi temperature sensor
//#define _LIQUIDLEVEL			// optical liquid level sensor
//#define _GEOLOCATION			// NeoBlox GPS
//#define _SWITCHES				// binary values from up to 8 input switches
//#define _FARMCASTER			// a special mote that produces audible alerts using MP3 files and player/amplifier/speaker

//#define _UNIT_TEST
//#define _TEST_EEPROM
//#define _TEST_WATCHDOG				// the MPU will enter an endless loop during EV_TXCOMPLETE event processing causing the watchdog to fire.

#if defined _SWITCHES
 int8_t input_pins[8]  = {-1, -1, -1, -1, -1, -1, -1, -1};			// -1 indicates no pin affinity
#endif

#if defined _BME680
  #define _BME680_USE_HW_SPI
#endif
#if defined _BME680_CT_PROFILER
  #define _BME680_CT_PROFILER_TEST
  #define _BME680_USE_HW_SPI
  #define _MAINS_SENSOR_1 10
#endif

#if defined _GEOLOCATION
  #define _ACCELERATION 20				// accelerate send rate for RFSURVEYS
#else
  #define _ACCELERATION 20				// set to 1 for most production sensor types
#endif

#if defined(_FARMCASTER)
#include <ArduinoJson.h>
#include <MsgPack.h>
#include <DFRobotDFPlayerMini.h>
DFRobotDFPlayerMini player;					// the Player object
uint8_t source = DFPLAYER_DEVICE_SD;
uint8_t playlist[64];
uint8_t playindx;
long lastdownlinkmillis;
#define _DOWNLINK_TIMEOUT_MILLIS 10 * 60000
#define _DFPLAYER_BUSY 13
#endif

//special pins for all motes
#define _BATTERY_PIN A7					// divider circuit internally wired here
#define _WATCHDOG_DONE 12				// pulsed once on each LoRa TX_COMPLETE event
#define _LOOP_PULSE_PIN 5				// pulsed once on each iteration of loop()
#define _BUZZER_PIN A3					// used for _LIQUID_LEVEL and _GEOLOCATION sensortypes only

const unsigned int TX_INTERVAL = _SEND_INTERVAL_MINUTES*60/_ACCELERATION;         	// Schedule the interval(seconds) at which data is sent
//static uint8_t data[_MAX_DATA_SIZE];
static osjob_t sendjob;

//#14 reset the processor when frame count reaches a threshold
unsigned int frameCount;
#define _FRAMECOUNT_MAX 500				// will reset when this count is reached
boolean ackRequested;
boolean ackReceived;

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

/*
 * LoRa Radio Pin Mappings
 */
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3,6},
}; //.dio = {3,6,11},

RTCZero rtc;

/*
 * Mote Record Structures
 */
#if defined(_SWITCHES)
  struct moterecord {
	    uint8_t batteryvoltage;
	    uint8_t switch_states;
        uint8_t lastackreceived;
   };
#elif defined(_BME680)
    #define _BME680_I2C_ADDR 0x77
    #define _DEVICE_PROFILE 36
    #include "Adafruit_Sensor.h"
    #include "Adafruit_BME680.h"
    #define SEALEVELPRESSURE_HPA (1013.25)
    struct moterecord {
	    uint8_t batteryvoltage;
        uint8_t temperature;
        uint8_t humidity;
        uint8_t pressure;
        uint8_t gasresistance;
        uint8_t lastackreceived;
    };
#if defined(_BME680_USE_HW_SPI)
    #define _BME680_CS 13			// was A5
    Adafruit_BME680 bme(_BME680_CS);
#else
    Adafruit_BME680 bme;   // I2C
#endif

#elif defined(_TMP36)
    #define _DEVICE_PROFILE 36
    #define _TMP36_PIN A5
    struct moterecord {
    	float lastackreceived;
        float batteryvoltage;
        float temperature;
    };
#elif defined(_18B20)
    #define _DEVICE_PROFILE 41
    #include <DS18B20.h>
    DS18B20 ds(A0);
    struct moterecord {
    	float batteryvoltage;
        float temperature;
        uint8_t  lastackreceived;
    };
#elif defined(_LIQUIDLEVEL)
    #define _DEVICE_PROFILE 39
    #define _LIQUIDLEVEL_A2D_PIN A1
    #define _LIQUIDLEVEL_IREMITTER_PIN A2			// GPIO9 is ~A7 Battery Pin
    #define _LIQUIDLEVEL_TEST_PIN 10
    struct moterecord {
    	uint8_t batteryvoltage;
        uint8_t darkmean;
        uint8_t darksd;
        uint8_t cycledmean;
        uint8_t cycledsd;
        uint8_t lastackreceived;
    };
#elif defined(_GEOLOCATION)
    #include "NMEAGPS.h"
    #include <GPSport.h>
    NMEAGPS  gps; // This parses the GPS characters
    gps_fix  fix; // This holds on to the latest values
    #define _DEVICE_PROFILE 40
    #define _MARCOPOLO_PIN 10               // INPUT_PULLUP HERE, used to trigger Slack notifications
    struct moterecord {
    	float lastackreceived;
        float batteryvoltage;
        float latitude;
        float longitude;
        float altitude;
        float marcopolo;		//used to trigger a Slack message
    };
#elif defined(_BME680_CT_PROFILER)
    #define _BME680_I2C_ADDR 0x77
    #define _DEVICE_PROFILE 41
    #include "Adafruit_Sensor.h"
    #include "Adafruit_BME680.h"
    #define SEALEVELPRESSURE_HPA (1013.25)
    struct moterecord {
    	uint8_t batteryvoltage;
    	uint8_t temperature;
    	uint8_t humidity;
    	uint8_t pressure;
    	uint8_t gasresistance;
        uint8_t current_1;
        uint8_t current_2;
        uint8_t current_3;
        uint8_t current_4;
        uint8_t current_5;
        uint8_t mainssensor_1;
        uint8_t lastackreceived;
    };
    #if defined(_BME680_USE_HW_SPI)
      #define _BME680_CS 13  //was A5
      Adafruit_BME680 bme(_BME680_CS);
    #else
      Adafruit_BME680 bme;   // I2C
    #endif

#else
    struct moterecord {    // a 'minimal' mote sends its' battery voltage
        float batteryvoltage;
        uint8_t lastackreceived;
    };
#endif

typedef struct moterecord Record;
Record moteRec;
//union moterecord moteRec;
char b[sizeof(moteRec)];
// [END] Mote Record Structures

#if defined(_LIQUIDLEVEL)
#define _WETDRY_WINDOWSIZE 20
uint8_t wetDryStatus[_WETDRY_WINDOWSIZE];

void readLevelSensor(Record *moteRecord) {
  #define  _LIQUIDSENSOR_SAMPLES 10
  unsigned int samples[_LIQUIDSENSOR_SAMPLES];
  static uint8_t wetDryIdx = 0;

  //read the sensor with DARK EMITTER
  digitalWrite(_LIQUIDLEVEL_IREMITTER_PIN, LOW);
  long meanDarkValue = 0;
  for(uint8_t n=0; n<_LIQUIDSENSOR_SAMPLES; n++){
    unsigned int value = analogRead(_LIQUIDLEVEL_A2D_PIN);
    meanDarkValue += value;
    samples[n] = value;
  }
  //calculate the mean
  meanDarkValue /= _LIQUIDSENSOR_SAMPLES;

  //calculate the standard deviation
  long meanSquared = 0;
  for(uint8_t n=0; n<_LIQUIDSENSOR_SAMPLES; n++) {
    meanSquared += sq(samples[n] - meanDarkValue);
  }
  moteRecord->darksd = (uint8_t) sqrt(meanSquared/_LIQUIDSENSOR_SAMPLES)/4;
  meanDarkValue = (meanDarkValue-3)/4;
  moteRecord->darkmean = (uint8_t) meanDarkValue;

  //read the sensor while CYCLING the EMITTER
  long meanCycledValue = 0;
  for(uint8_t n=0; n<_LIQUIDSENSOR_SAMPLES; n++){
	 if ((n&0x01)==1)
       digitalWrite(_LIQUIDLEVEL_IREMITTER_PIN, HIGH);
	 else
	   digitalWrite(_LIQUIDLEVEL_IREMITTER_PIN, LOW);
	 delay(1);
     unsigned int value = analogRead(_LIQUIDLEVEL_A2D_PIN);
     meanCycledValue += value;
     samples[n] = value;
  }
  //make sure the emitter is disabled
  digitalWrite(_LIQUIDLEVEL_IREMITTER_PIN, LOW);

  //calculate the mean
  meanCycledValue /= _LIQUIDSENSOR_SAMPLES;

  //calculate the standard deviation
  meanSquared = 0;
  for(uint8_t n=0; n<_LIQUIDSENSOR_SAMPLES; n++) {
    meanSquared += sq(samples[n] - meanCycledValue);
  }
  moteRecord->cycledsd  = (uint8_t) sqrt(meanSquared/_LIQUIDSENSOR_SAMPLES)/4;
  meanCycledValue = (meanCycledValue-3)/4;
  moteRecord->cycledmean = (uint8_t) meanCycledValue;
  #ifdef _DEBUG_SERIAL
  Serial.print("Dark V  "); Serial.print(moteRecord->darkmean); Serial.print(","); Serial.println(moteRecord->darksd);
  Serial.print("Cycled "); Serial.print(moteRecord->cycledmean); Serial.print(","); Serial.println(moteRecord->cycledsd);
  #endif
}
#endif

#if defined(_BME680) or defined(_BME680_CT_PROFILER)
  void readEnvironmentSensor(Record *moteRecord) {
    #if defined(_I2C_WATCHDOG)
    // set the watchdog to reset the MPU in ~4000ms
    Watchdog.enable(4000);
    #endif

    #define _BME680_AVERAGING_SAMPLES 1

    float temperature = 0.0;
    float humidity = 0.0;
    float pressure = 0.0;
    float gasresistance = 0.0;

    for(uint8_t n=0; n<_BME680_AVERAGING_SAMPLES; n++) {
      bme.performReading();
      temperature +=  bme.temperature;
      humidity += bme.humidity;
      pressure += bme.pressure;
      gasresistance += bme.gas_resistance;
    }
    #if defined(_I2C_WATCHDOG)
      // if we've made it here we can disable the watchdog reset.
      Watchdog.disable();
    #endif
    // add 0.5 before casting to uint8_t
    moteRecord->temperature = (uint8_t)( (( (temperature/_BME680_AVERAGING_SAMPLES) + 25.333333)/0.275817) + 0.5);
    moteRecord->humidity = (uint8_t) 2 * (humidity/_BME680_AVERAGING_SAMPLES);
    moteRecord->pressure =  (uint8_t) (pressure/_BME680_AVERAGING_SAMPLES) - 900.0;
    moteRecord->gasresistance = (uint8_t) (gasresistance/_BME680_AVERAGING_SAMPLES)/2.0;
    #if defined(_DEBUG_SERIAL)
    Serial.print("\nTemperature = ");
    Serial.println((9.0 * (moteRecord->temperature*0.275817-25.3333)/5.0 + 32.0));
    Serial.print("Pressure = ");
    Serial.println((1.0*((900 + moteRecord->pressure))));
    Serial.print("Humidity = ");
    Serial.println(moteRecord->humidity * 0.5);
    Serial.print("Gas = ");
    Serial.println(moteRecord->gasresistance * 2000);
    Serial.println();
    #endif

}
#endif

#if defined (_BME680_CT_PROFILER)
  #define _MAINS_SAMPLES 10
  void readMainsSensors(Record *moteRecord) {
	  /*
	   * 0 - Mains Ok
	   * 1 - Mains Bad
	   */
	  uint8_t mains_1 = 0;
	  for (uint8_t n=0; n<_MAINS_SAMPLES; n++) {
		mains_1 += digitalRead(_MAINS_SENSOR_1);
		delay(10);
	  }
	  moteRecord->mainssensor_1 = (uint8_t) mains_1;
      #if defined(_DEBUG_SERIAL)
   	  Serial.print("MAINSENSOR_1 ="); Serial.println(mains_1);
      #endif
  }
  void readCurrentTransformers(Record *moteRecord) {
      #if defined(_I2C_WATCHDOG)
      // set the watchdog to reset the MPU in ~4000ms
      Watchdog.enable(4000);
      #endif

      #define _CT_AVERAGING_SAMPLES 64
      #define ADC_REF 3.3

      float current[5];
      float voltage[5];
      long adcReading[5];
      unsigned int adc;

      #if defined(_DEBUG_SERIAL)
        Serial.println("=====");
      #endif

      adcReading[0] += analogRead(A1);
      adcReading[1] += analogRead(A2);
      adcReading[2] += analogRead(A3);
      adcReading[3] += analogRead(A4);
      adcReading[4] += analogRead(A5);
      adcReading[0] = 0;
      adcReading[1] = 0;
      adcReading[2] = 0;
      adcReading[3] = 0;
      adcReading[4] = 0;
      for (uint8_t n=0; n< _CT_AVERAGING_SAMPLES; n++) {    // Perform averaging
    	adcReading[0] += analogRead(A1);
    	adcReading[1] += analogRead(A2);
    	adcReading[2] += analogRead(A3);
    	adcReading[3] += analogRead(A4);
    	adcReading[4] += analogRead(A5);
    	delay(1);
      }
      for (uint8_t ch=0; ch<5; ch++) {
    	  adcReading[ch] /= _CT_AVERAGING_SAMPLES;
    	  /*
    	   * Rated voltage output from CT signal conditioner is 0.2 -> 2.8VDC
    	   * Howeverm the observed output for 0A primary current is 0.0VDC
    	   * We'll use the 3.3v default reference for ADC
    	   * If current < 100mA we will indicate 0.0A
    	   * We'll calculate the RMS value
    	   */
    	  voltage[ch] = ADC_REF * adcReading[ch]/1023;
    	  current[ch] = (voltage[ch] * 20.0/2.8) * 0.707;
    	  if (current[ch]<0.1)
    		current[ch] = 0.0;
      }

      #if defined(_DEBUG_SERIAL)
   	  Serial.print("I(A1)  ="); Serial.print(adcReading[0]); Serial.print(" "); Serial.print(voltage[0]); Serial.print("V "); Serial.print(current[0]); Serial.println(" A(rms)");
   	  Serial.print("I(A2)  ="); Serial.print(adcReading[1]); Serial.print(" "); Serial.print(voltage[1]); Serial.print("V "); Serial.print(current[1]); Serial.println(" A(rms)");
   	  Serial.print("I(A3)  ="); Serial.print(adcReading[2]); Serial.print(" "); Serial.print(voltage[2]); Serial.print("V "); Serial.print(current[2]); Serial.println(" A(rms)");
   	  Serial.print("I(A4)  ="); Serial.print(adcReading[3]); Serial.print(" "); Serial.print(voltage[3]); Serial.print("V "); Serial.print(current[3]); Serial.println(" A(rms)");
   	  Serial.print("I(A5)  ="); Serial.print(adcReading[4]); Serial.print(" "); Serial.print(voltage[4]); Serial.print("V "); Serial.print(current[4]); Serial.println(" A(rms)");
   	  //Serial.print("I(2x  A2)  ="); Serial.print(adcReading[1]); Serial.print(" "); Serial.print(voltage[1]); Serial.print("V "); Serial.print(current[1]); Serial.print(" A(rms)"); Serial.print(" ");  Serial.println(current[1]/current[0]);
   	  //Serial.print("I(10x A1)  ="); Serial.print(adcReading[0]); Serial.print(" "); Serial.print(voltage[0]); Serial.print("V "); Serial.print(current[0]); Serial.print(" A"); Serial.print(" ");  Serial.println(current[0]/current[2]);
      #endif

      #if defined(_I2C_WATCHDOG)
        // if we've made it here we can disable the watchdog reset.
        Watchdog.disable();
      #endif

      moteRecord->current_1 = (uint8_t)(adcReading[0]>>2);
      moteRecord->current_2 = (uint8_t)(adcReading[1]>>2);
      moteRecord->current_3 = (uint8_t)(adcReading[2]>>2);
      moteRecord->current_4 = (uint8_t)(adcReading[3]>>2);
      moteRecord->current_5 = (uint8_t)(adcReading[4]>>2);
  }
#endif

#if defined(_SWITCHES)
  void readSwitches(Record *moteRecord) {
	  uint8_t mask = 0x01;
	  uint8_t switchByte = 0x00;
	  for(uint8_t n=0; n<8; n++) {
		  if (digitalRead(input_pins[n])==HIGH)
			  switchByte = switchByte & mask;
		  mask = mask<<1;
	  }
	 moteRecord->	switch_states = switchByte;
  }
#endif

/* EEPROM I2C Device address calculated as follows:
 * 101 0[A2][A1][A0]
 *
 * DEVEUI the EUI-64 address is stored in the EEPROM at 0xF8
 * APPKEY is constructed from the APP_KEY_ROOT with the 3LSB replaced by the 3LSB of the DEVEUI
 *                                            s     e     n     s     e     i     0     0
 */
static const u1_t  	APPKEY_ROOT[16] 	= {0x73, 0x65, 0x6E, 0x73, 0x65, 0x69, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u1_t 	PROGMEM APPEUI[8]  	= {0x22, 0xf6, 0xec, 0x6b, 0x9a, 0x9a, 0x62, 0xdf};

u1_t devEui[8];
u1_t appKey[16];

void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write(eeaddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
       if (Wire.available())
         buffer[c] = Wire.read();
    Wire.endTransmission();
}

// These methods are called by the LMIC code after the JOIN
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, devEui, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, appKey, 16);
}

#if defined(_TEST_EEPROM)

void EEPROM_write(uint16_t addr,uint8_t data) {
  Wire.beginTransmission(_EEPROM_IC2_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();    // stop transmitting
}

byte EEPROM_read(uint16_t addr) {
  byte data;
  Wire.beginTransmission(_EEPROM_IC2_ADDR);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(_EEPROM_IC2_ADDR, 1);

  if (Wire.available()) {
    data = Wire.read();
  }
  Wire.endTransmission();

  return data;
}

void testEEPROM() {
  //test the I2C EEPROM 2AA02E64
  for(uint8_t addr=0; addr<0xFF; addr++) {
	  EEPROM_write(addr, (byte) addr);
	  byte read = EEPROM_read(addr);
	  Serial.print("at address "); Serial.print(addr); Serial.print(" we wrote "); Serial.print(addr); Serial.print(" and read "); Serial.println(read);
  }
}
#endif

/*
 * do_send() is called periodically to send sensor data
 */
void do_send(osjob_t* j){
	Serial1.println("sending..");
    if (LMIC.opmode & OP_TXRXPEND) {  // Check if there is a current TX/RX job running

    } else {
#if defined(_USE_NEOPIXEL)
	  pixels.setPixelColor(0, pixels.Color(255,255,0));
      pixels.show();
      delay(10);
      pixels.setPixelColor(0, pixels.Color(0,0,0));
      pixels.show();
      delay(10);
#endif
      #if defined(_BME680)
      readEnvironmentSensor(&moteRec);
      #elif defined(_BME680_CT_PROFILER)
      readEnvironmentSensor(&moteRec);
      readCurrentTransformers(&moteRec);
      readMainsSensors(&moteRec);
      #elif defined(_TMP36)
      moteRec.temperature = analogRead(_TMP36_PIN) * 3.3 /1024;
      #elif defined(_18B20)

      while (ds.selectNext()) {
        Serial.print("DS18B20 ");
        //Serial.println(ds.getTempF());  //32.0 + 9.0 * (temperature/_BME680_AVERAGING_SAMPLES)/5.0;
        moteRec.temperature = ds.getTempF();
      }
      //moteRec.temperature = 56.7;
      Serial.print("T="); Serial.println(moteRec.temperature);
      #elif defined(_LIQUIDLEVEL)
      readLevelSensor(&moteRec);
      #elif defined(_GEOLOCATION)
      gpsPort.begin(9600);
      while (!gps.available(gpsPort));
      fix = gps.read();
      if (fix.valid.location) {
        moteRec.latitude  = fix.latitude();
        moteRec.longitude = fix.longitude();
        if (fix.valid.altitude)
          moteRec.altitude =  fix.altitude();
        else
          moteRec.altitude = 0.0;
        for(uint8_t n=0; n<2; n++) {
       	  digitalWrite(_BUZZER_PIN, HIGH);			// beep twice to indicate GPS lock
       	  delay(75);
       	  digitalWrite(_BUZZER_PIN, LOW);
       	  delay(75);
        }
      } else {
        digitalWrite(_BUZZER_PIN, HIGH);					// beep once only to indicate GPS is not locked
        delay(75);
        digitalWrite(_BUZZER_PIN, LOW);
        delay(75);
      }
      moteRec.marcopolo = (float) digitalRead(_MARCOPOLO_PIN);
      #elif defined(_SWITCHES)
      readSwitches(&moteRec);
      #endif

      if (ackReceived)
        moteRec.lastackreceived = (uint8_t) 1;
      else
        moteRec.lastackreceived = (uint8_t) 0;

      //determine an average battery voltage
      #define _BATTERY_AVERAGING_SAMPLES 10
      float batteryVoltage;
      long adcReading;
      adcReading = analogRead(_BATTERY_PIN);
      adcReading = 0;                                 // Discard inaccurate 1st reading
      for (uint8_t n=0; n< _BATTERY_AVERAGING_SAMPLES; n++) {    // Perform averaging
        adcReading += analogRead(_BATTERY_PIN);
      }
      adcReading /= _BATTERY_AVERAGING_SAMPLES;
      batteryVoltage = (6.6 * adcReading)/1024;		// resistor divider is 50%, use 2x the 3.3v reference voltage
      //cp 3/22 moteRec.batteryvoltage = (batteryVoltage-2.45) * 100; //batteryVoltage;
      moteRec.batteryvoltage = batteryVoltage;			 //batteryVoltage;

      memcpy(b, &moteRec, sizeof(moteRec));

      #ifdef _ACK_PACKETS
      if ( ((frameCount%5)==1) ||  ((frameCount%5)==3) ) {
        LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 1);	// queue the packet, ACK
        ackRequested = true;
        ackReceived = false;
      } else {
        LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 0);	// queue the packet, NO ACK
        ackRequested = false;
        ackReceived = false;
      }
      #else
      LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 0);	// queue the packet, no ACK
      ackRequested = false;
      ackReceived = false;
      #endif

      #ifdef _DEBUG_SERIAL
      const byte * p = (const byte*) &moteRec;
      for (uint8_t n=0; n<sizeof(moteRec); n++) {
        Serial.print((byte) p[n], HEX); Serial.print(" ");
      }

      if (ackRequested)
        Serial.println(F("Confirmed packet queued for transmission."));
      else
        Serial.println(F("Unconfirmed packet queued for transmission."));
      #endif
    }
}

void sitUbu() {
  #if not defined(_TEST_WATCHDOG)
  digitalWrite(_WATCHDOG_DONE, HIGH);				// rising edge pulse on DONE to keep the watchdog happy!
  delay(2);											// pet the dog before going to sleep.
  digitalWrite(_WATCHDOG_DONE, LOW);
  #endif
}

/* Issue #2
 * alarmMatch() is invoked when the rtcZero clock alarms.
 */
void alarmMatch()  {
  // Enable systick interrupt
  //SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  //#14 reset on _FRAMECOUNT_MAX
  if (frameCount>=_FRAMECOUNT_MAX) {
	NVIC_SystemReset();      // processor software reset, the scheduled packet will be delayed slightly.
  } else {
	//the following call attempts to avoid collisions with other motes by randomizing the send time
	os_setTimedCallback(&sendjob, os_getTime()+ms2osticks(random(2000)), do_send);
	frameCount++;
  }
}

void goToSleep() {
  rtc.setAlarmEpoch( rtc.getEpoch() + TX_INTERVAL);    // Sleep for a period of TX_INTERVAL using single shot alarm
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  rtc.attachInterrupt(alarmMatch);
  // Disable systick interrupt
  //SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  delay(10);
  /*
   * ZZZZZzzzzzzz (we're sleeping right here!)
   */
  #if not defined (_DONT_SLEEP)
    rtc.standbyMode();                                  // Enter sleep mode
  #endif
}

void onEvent (ev_t ev) {
  #ifdef _DEBUG_SERIAL
  Serial.print(os_getTime());
  Serial.print(": ["); Serial.print(ev); Serial.print("] ");
  #endif
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_SCAN_TIMEOUT"));
      #endif
      break;
    case EV_BEACON_FOUND:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_BEACON_FOUND"));
      #endif
      break;
    case EV_BEACON_MISSED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_BEACON_MISSED"));
      #endif
      break;
    case EV_BEACON_TRACKED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_BEACON_TRACKED"));
      #endif
      break;
    case EV_JOINING:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_JOINING"));
      #endif
      break;
    case EV_JOINED: {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_JOINED"));
      Serial.print("netid: ");
      Serial.println(netid, DEC);
      Serial.print("devaddr: "); // @suppress("Method cannot be resolved")
      Serial.println(devaddr, HEX);
      Serial.print("artKey: ");
      for (int i=0; i<sizeof(artKey); ++i) {
        if (i != 0)
          Serial.print("-");
        Serial.print(artKey[i], HEX);
      }
      Serial.println("");
      Serial.print("nwkKey: ");
      for (int i=0; i<sizeof(nwkKey); ++i) {
        if (i != 0)
          Serial.print("-");
        Serial.print(nwkKey[i], HEX);
      }
      Serial.println("");
      #endif
      LMIC_setLinkCheckMode(0);
      break;
    }
    case EV_JOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_JOIN_FAILED"));
      #endif
      goToSleep();
      break;
    case EV_REJOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_REJOIN_FAILED"));
      #endif
      goToSleep();
      break;
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);     // LED Off
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      #endif
      if ((ackRequested) && (LMIC.txrxFlags & TXRX_ACK)) {
        // if confirmation was requested and is received - cycle the WATCHDOG bit
        ackReceived = true;
#if defined(_USE_NEOPIXEL)
	    pixels.setPixelColor(1, pixels.Color(0,255,0));
        pixels.show();
        delay(10);
        pixels.setPixelColor(1, pixels.Color(0,0,0));
        pixels.show();
        delay(10);
#endif
        sitUbu();
        #ifdef _DEBUG_SERIAL
        Serial.println(F("Confirmed - Ok!"));
        #endif
      }
      // Check if we have a downlink on either Rx1 or Rx2 windows
      if ((LMIC.txrxFlags & ( TXRX_DNW1 | TXRX_DNW2 )) != 0) {
      	if ((LMIC.txrxFlags & TXRX_DNW1) != 0)
       	  Serial.print(F("Downlink on Rx1-"));
       	else
       	  Serial.print(F("Downlink on Rx2-"));

       	if (LMIC.dataLen) {
       	  #ifdef _DEBUG_SERIAL
       	  Serial.print(LMIC.dataLen);
       	  Serial.println(F(" bytes:"));
       	  // Receive bytes in LMIC.frame are the B64 decode of the
       	  // 'data' field sent by ChirpStack
       	  for (int i = 0; i < LMIC.dataLen; i++) {
       	     Serial.print(" 0x"); Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	  }
       	  Serial.println();
       	  #endif
#ifdef _FARMCASTER
       	  StaticJsonDocument<640> doc;
       	  MsgPack::Unpacker unpacker;
       	  unpacker.feed(&LMIC.frame[LMIC.dataBeg], LMIC.dataLen);

       	  unpacker.deserialize(doc);
       	  if (doc["cmd"]==1) {  //Playlist
#ifdef _DEBUG_SERIAL
           Serial.println(F("Playlist"));
#endif
           uint8_t n=0;
           do {
       	     playlist[n] = doc["arg"][n];
       	     Serial.println(playlist[n]);
       	     n++;
           } while (doc["arg"][n] != 0);
           playlist[n] = 0;
           playlist[n+1] = 0;
           playlist[n+2] = 0;
           playindx = 0;
           lastdownlinkmillis = millis();
       	  }
#endif
       	} else {
          #ifdef _DEBUG_SERIAL
       	  Serial.println();
          #endif
       	}
      }

      #ifdef _GEOLOCATION
      //pinMode(_BUZZER_PIN, OUTPUT);
      digitalWrite(_BUZZER_PIN, HIGH);					// beep once to indicate reception
      delay(75);
      digitalWrite(_BUZZER_PIN, LOW);
      #endif

      #ifdef _DEBUG_SERIAL
      #if not defined (_DONT_SLEEP)
      Serial.flush();                                     // Ensure all debugging messages are sent before sleep
      USBDevice.detach();                                 // USB port consumes extra current
      #endif
      #endif
      goToSleep();
      #ifdef _DEBUG_SERIAL
      #if not defined (_DONT_SLEEP)
      USBDevice.init();                                   // Reinitialize USB for debugging
      USBDevice.attach();
      #endif
      #endif
      break;
    case EV_LOST_TSYNC:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_LOST_TSYNC"));
      #endif
      break;
    case EV_RESET:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_RESET"));
      #endif
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_RXCOMPLETE"));
      #endif
      break;
    case EV_LINK_DEAD:
      //no confirmation has been received for an extended perion
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_LINK_DEAD"));
      #endif
      goToSleep();
      break;
    case EV_LINK_ALIVE:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_LINK_ALIVE"));
      #endif
      break;
    case EV_TXSTART:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_TXSTART"));
      #endif
      //digitalWrite(LED_BUILTIN, LOW);
      break;
    default:
      #ifdef _DEBUG_SERIAL
      Serial.print(F("Unknown EV: "));
      Serial.println((unsigned) ev);
      #endif
      break;
  }
}

void setup() {
  // Resets occurring ~immediately after a successful SEND #10, is this the Brown Out Detector firing?
  // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50 uA
  SYSCTRL->VREG.bit.RUNSTDBY = 1;

  /*
   * Place all pins in default state initially, specific pins are reconfigured per application below
   */

  pinMode(0, INPUT_PULLUP);                            // RX and TX
  pinMode(1, INPUT_PULLUP);

  unsigned char pinNumber;
  for (pinNumber = 8; pinNumber <= 22; pinNumber++) {
    pinMode(pinNumber, INPUT_PULLUP);
  }

  pinMode(25, INPUT_PULLUP);                           // RX_LED (D25) & TX_LED (D26) (both LED not mounted on Mini Ultra Pro)
  pinMode(26, INPUT_PULLUP);
  pinMode(30, INPUT_PULLUP);                           // D30 (RX) & D31 (TX) of Serial
  pinMode(31, INPUT_PULLUP);

  for (pinNumber = 34; pinNumber <= 38; pinNumber++) { // D34-D38 (EBDG Interface)
    pinMode(pinNumber, INPUT_PULLUP);
  }


  // Configure built-in LED -- will flash when sending
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t m=0; m<100; m++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    digitalWrite(LED_BUILTIN, LOW);
    delay(20);
  }

  pinMode(_WATCHDOG_DONE, OUTPUT);					 // Watchdog Done pin
  digitalWrite(_WATCHDOG_DONE, LOW);
  digitalWrite(_WATCHDOG_DONE, LOW);

  pinMode(_LOOP_PULSE_PIN, OUTPUT);

#if defined(_USE_NEOPIXEL)
  pixels.setBrightness(255);
  pixels.begin();
#if defined(_TEST_NEOPIXEL)
  uint8_t r =0;
  uint8_t g =0;
  uint8_t b =0;
  do {
	  for(uint8_t n=0; n<NUMPIXELS; n++) {
	    pixels.setPixelColor(n, pixels.Color(r,g,b));
	    pixels.show();
	    delay(10);
	    pixels.setPixelColor(n,pixels.Color(0,0,0));
	    pixels.show();
	    delay(10);
	    r+=10;
	    g+=20;
	    b+=30;
	  }
  } while(true);
#endif
#endif

  #ifdef _DEBUG_SERIAL
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
#if defined(_USE_NEOPIXEL)
    pixels.setPixelColor(0, pixels.Color(255,0,0));
    pixels.show();
    delay(10);
    pixels.setPixelColor(0, pixels.Color(0,0,0));
    pixels.show();
    delay(10);
#endif
  }
  Serial.println(F("Starting"));
  Serial.println("Reading EEPROM...");
  #else
  USBDevice.detach();                                  // Detach USB interface if not in _DEBUG_SERIAL mode #9
  #endif
  // are lockups occurring on the I2C bus?
  Wire.begin();
  //Wire.setClock(100000UL);
  Wire.setClock(50000UL);			// lowering the clock frequency to see if reliability improves
  //get the DEVEUI from the EEPROM
  i2c_eeprom_read_buffer(_EEPROM_IC2_ADDR, _DEVEUI_ADDR, devEui, 8);
  memcpy_P(appKey, APPKEY_ROOT, 16);
  //modify the appKey
  for (uint8_t n=8; n<16; n++) {
    appKey[n] = devEui[n-8];
  }

  #if defined(_DUMP_KEYS)
  delay(1000);
  Serial.println("EEPROM read complete.");
  char hexdigit[2];
  Serial.print("coreId (DB): ");
  for(int n=0; n<8; n++) {
    sprintf(hexdigit,"%02x",devEui[n]);
    Serial.print(hexdigit);
  }
  Serial.print("\n\rdevEui (Chirpstack format): ");
  for(int n=7; n>=0; n--) {
    sprintf(hexdigit,"%02x",devEui[n]);
    Serial.print(hexdigit);
  }
  Serial.print("\n\rappKey: ");
  for(uint8_t n=0; n<16; n++) {
    sprintf(hexdigit,"%02x",appKey[n]);
    Serial.print(hexdigit);
  }
  Serial.println();
  #endif

  rtc.begin();                                        // Initialize RTC
  rtc.setEpoch(0);                                    // Use RTC as a second timer instead of calendar


#if defined(_FARMCASTER)
    pinMode(_DFPLAYER_BUSY, INPUT);
    // Init serial port for DFPlayer Mini
    Serial1.begin(9600);
    //delay(2000);
    // Start communication with DFPlayer Mini
    if (player.begin(Serial1)) {
      Serial.println("DfPlayer OK");
      // Set volume to maximum (0 to 30).
      player.volume(30);
      // Play the first MP3 file on the SD card
      player.outputDevice(source);
      player.playFolder(7,1);
    } else {
      Serial.println("DfPlayer Not Found");
    }
#if defined(_FARMCASTER_TEST)
    do {
    for(uint8_t n=1; n<=16; n++) {
    	while(digitalRead(_DFPLAYER_BUSY)==LOW) {}
    	if (digitalRead(_DFPLAYER_BUSY)==HIGH) {
    	      player.playFolder(1, n);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
    	      while(digitalRead(_DFPLAYER_BUSY)==HIGH) {
    	    	// wait until DFPlayer accepts command
    	      }
    	}
    }
    } while(true);
#endif

#endif

  #if defined(_BME680)
  if (!bme.begin()) {
    #ifdef _DEBUG_SERIAL
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    #endif
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_2X);       // Set up oversampling and filter initialization
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(20, 15); // 20*C for 15 ms
  #if defined(_UNIT_TEST)
  unsigned long testcount = 0;
  do {
    #ifdef _DEBUG_SERIAL
    Serial.println(testcount++);
    #endif
    readEnvironmentSensor(&moteRec);
    sitUbu();
  } while(1);
  #endif
  #ifdef _DEBUG_SERIAL
  Serial.println("BME680 initialized.");
  #endif
  #elif defined(_LIQUIDLEVEL)
  pinMode(_BUZZER_PIN, OUTPUT);
  digitalWrite(_BUZZER_PIN, LOW);
  pinMode(_LIQUIDLEVEL_IREMITTER_PIN, OUTPUT);
  pinMode(_LIQUIDLEVEL_TEST_PIN, INPUT_PULLUP);
  delay(1000);
  Record moteRec;

  do {
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(_BUZZER_PIN, HIGH);
	readLevelSensor(&moteRec);
	uint8_t nbeeps = 1;
	if (moteRec.cycledmean > 650)
	  nbeeps = 2;	// WET
    for (uint8_t n=0; n<nbeeps; n++) {
	  digitalWrite(_BUZZER_PIN, HIGH);					// beep to indicate reception
	  delay(50);
	  digitalWrite(_BUZZER_PIN, LOW);
	  delay(50);
	}

	Serial.println();
	delay(1000);
  } while (digitalRead(_LIQUIDLEVEL_TEST_PIN)==LOW);

  digitalWrite(LED_BUILTIN, LOW);
  #elif defined(_GEOLOCATION)
  pinMode(_MARCOPOLO_PIN, INPUT_PULLUP);
  pinMode(_BUZZER_PIN, OUTPUT);
  for(uint8_t n=0; n<2; n++) {
	digitalWrite(_BUZZER_PIN, HIGH);					// beep to indicate reception
	delay(150);
	digitalWrite(_BUZZER_PIN, LOW);
	delay(150);
  }
  digitalWrite(_BUZZER_PIN, LOW);
  gpsPort.begin(9600);
  fix = gps.read();
  #endif

  #if defined(_BME680_CT_PROFILER)
  pinMode(_MAINS_SENSOR_1, INPUT_PULLUP);
  if (!bme.begin()) {
      #ifdef _DEBUG_SERIAL
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
      #endif
      while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_2X);       // Set up oversampling and filter initialization
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(20, 15); // 20*C for 15 ms
  #ifdef _DEBUG_SERIAL
  Serial.println("BME680 initialized.");
  #endif

  analogReference(AR_DEFAULT);
  #ifdef _UNIT_TEST
  do {
	readEnvironmentSensor(&moteRec);
	readCurrentTransformers(&moteRec);
	delay(1000);
  } while(1);
  #endif
  #endif

#if defined(_SWITCHES)
  for (uint8_t n=0; n<8; n++) {
	  if (input_pins[n] >=0)
		  pinMode(input_pins[n], INPUT_PULLUP);
  }
#endif



  os_init();                                               // LMIC init
  LMIC_reset();                                            // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  #if defined(_FIXED_CHANNEL)
  /* CP testing
  for (int b = 0; b < 8; ++b) {
    LMIC_disableSubBand(b);
  }
  // Then enable the channel(s) you want to use
  LMIC_enableChannel(_FIXED_CHANNEL);
  */
  for(uint8_t c=0; c<72; c++) {
  	if (c!=_FIXED_CHANNEL)
      LMIC_disableChannel(c);
  	else
  	  LMIC_enableChannel(c);
  }
  #else
  LMIC_selectSubBand(1);
  #endif

  LMIC_setLinkCheckMode(0);
  //03/07/21 was LMIC_setDrTxpow(DR_SF10,14);
  LMIC_setDrTxpow(DR_SF7,14);


  #ifdef _DEBUG_SERIAL
  Serial.println("Setup completed.");
  #endif

  #if defined(_GEOLOCATION) and defined(_UNIT_TEST)
  while (true) {
	if (gps.available(gpsPort)) {
      fix = gps.read();

      Serial.print("Location: ");
      if (fix.valid.location) {
        Serial.print( fix.latitude(), 6 );
        Serial.print( ',' );
        Serial.println(fix.longitude(), 6 );
      }

      Serial.println(F(", Altitude: ") );
      if (fix.valid.altitude)
    	Serial.println(fix.altitude() );
	}
  }
  #endif

  #if defined(_TEST_EEPROM)
  testEEPROM();
  #endif

  randomSeed(analogRead(1));					//use the random number generate to randomize our sends

  //#14 initialize frameCount
  frameCount=0;
  Serial1.begin(9600);
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
  sitUbu();

#if defined(_FARMCASTER)
  //process the DfPlayer playlist
  if (digitalRead(_DFPLAYER_BUSY)==HIGH) {
    uint8_t folder = playlist[playindx*3];
    uint8_t file   = playlist[playindx*3 + 1];
    uint8_t volume = playlist[playindx*3 + 2];
    if (folder!=0) {
      player.volume(volume);
      player.playFolder(folder,file);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
      while(digitalRead(_DFPLAYER_BUSY)==HIGH) {
    	// wait until DFPlayer accepts command
      }
      playindx++;
    }
  }
  if ((millis() - lastdownlinkmillis) > _DOWNLINK_TIMEOUT_MILLIS) {
	  playlist[0] = 1;
	  playlist[1] = 2;
	  playlist[2] = 30;
	  playlist[3] = 0;
	  playlist[4] = 0;
	  playlist[5] = 0;
	  playindx = 0;
	  lastdownlinkmillis = millis();
  }
#endif
  digitalWrite(_LOOP_PULSE_PIN, HIGH);		// o'scope here to see if we're ALIVE!
  delay(1);
  digitalWrite(_LOOP_PULSE_PIN, LOW);
  delay(1);
  // #3 adding delay in loop() can cause numerous problems
  #if defined(_GEOLOCATION)
  fix = gps.read();
  #endif
}

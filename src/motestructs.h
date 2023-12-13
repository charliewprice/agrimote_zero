#ifndef MoteStructs__h
#define MoteStructs__h

/*
 * Mote Record Structures
 */

#if defined(_BME680)

    struct moterecord {
	    uint8_t batteryvoltage;
        uint8_t temperature;
        uint8_t humidity;
        uint8_t pressure;
        uint8_t gasresistance;
        uint8_t lastackreceived;
    };
    struct moteconfig {
          	uint16_t sendIntervalSecs;
    };

#elif defined(_TMP36)
    struct moterecord {
    	float lastackreceived;
        float batteryvoltage;
        float temperature;
    };
    struct moteconfig {
          	uint16_t sendIntervalSecs;
    };

#elif defined(_LIQUIDLEVEL)

    struct moterecord {
    	uint8_t batteryvoltage;
        uint8_t darkmean;
        uint8_t darksd;
        uint8_t cycledmean;
        uint8_t cycledsd;
        uint8_t lastackreceived;
    };
    struct moteconfig {
          	uint16_t sendIntervalSecs;
    };
#elif defined(_GEOLOCATION)

    struct moterecord {
    	float lastackreceived;
        float batteryvoltage;
        float latitude;
        float longitude;
        float altitude;
        float marcopolo;		//used to trigger a Slack message
    };
    struct moteconfig {
          	uint16_t sendIntervalSecs;
    };
#elif defined(_BME680_CT_PROFILER)

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
    struct moteconfig {
          	uint16_t sendIntervalSecs;
    };

#elif defined(_BME680_INTERVALOMETER)

    struct moterecord {
    	uint8_t batteryvoltage;
    	uint8_t temperature;
    	uint8_t humidity;
    	uint8_t pressure;
    	uint8_t gasresistance;
        uint8_t current_1;
        uint8_t ssr_1;
        uint8_t lastackreceived;
    };
    struct moteconfig {
       	uint8_t tempSetPoint;
   		uint8_t tempIdleBand;
   		uint8_t intervalPeriodSecs;
   		uint8_t intervalOnTimeSecs;
   		uint16_t sendIntervalSecs;
    };
#else
    struct moterecord {    // a 'minimal' mote sends its' battery voltage
        float batteryvoltage;
        uint8_t lastackreceived;
    };
    struct moteconfig {
      	uint16_t sendIntervalSecs;
    };
#endif

typedef struct moterecord Record;
typedef struct moteconfig Config;

#endif

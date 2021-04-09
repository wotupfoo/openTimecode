// --------------------------------------------
// HARDWARE Libraries
// --------------------------------------------
#define DEBUG 1
//#define ENABLE_WIFI 1

#ifdef DEBUG
#include "debug_scope.h"
#endif

// ESP32 HARDWARE PIN ASSIGNMENTS
#include "esp32_pins.h"

// !!!!!!! THIS IS NOT MULTI-THREADED !!!!!!!!!
// !!!!!!! YOU CAN'T USE IT ON CORE 1 !!!!!!!!!
#include <Wire.h> // I2C

// RTC header
#include <DS3231.h> // Realtime Clock (RTC) Temperature Compensated Oscillator(TCXO)

// --------------------------------------------
// SOFTWARE Functionality Libraries
// --------------------------------------------
// FreeRTOS headers
#include <FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Graphics headers
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef ENABLE_WIFI
#include <WiFiUdp.h>   // UDP
#include <NTPClient.h> // NTP Client https://github.com/arduino-libraries/NTPClient
#endif

// --------------------------------------------
// Non-Arduino Libraries
// --------------------------------------------
#include <encoder.h> // libLTC Encoder (timecode->audio)
#include <decoder.h> // libLTC Decoder (audio->timecode)

// --------------------------------------------
// My helper headers
// --------------------------------------------
#include "audiolevels.h" // Definitions of the LINE vs MIC levels in millivolts and dBu/dBv
#include "32kHztick_table.h"
//#include "fast_ssd1306.h"	// Speed up the update of the screen by only sending the delta

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// PRE-PROCESSOER DIRECTIVES
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// SERIAL port

// I2C PORT

// I2C OLED DISPLAY
#define SCREEN_ADDRESS 0x3C // I2C address
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels

// Timecode location (in pixels from top left) on the screen
#define TIMECODE_FONTSIZE 1 // 1 or 2
#define TIMECODE_X 36       // 60 = (SML_FONT_X*TIMECODE_INDENT)
#define TIMECODE_Y 16       // last line 56 // last-1 line 48 = (SML_FONT_Y*TIMECODE_LINE)

// RTC CLOCK
#define TICK_32kHz 32768
#define TICK_8kHz  8192

// Linear Time Code
#define LTC_PACKET_BITS 80
#define LTC_PACKET_BYTES 10
#define LTC_FPS_F_MAX 30.0
#define LTC_FPS_MAX 30
#define LTC_FPS_F_MIN 24.0
#define LTC_FPS_MIN 24
#define LTC_AUDIO_RATE TICK_32kHz

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// CLASSES & FORWARD DECLARATIONS
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void LTCtimecode_task(__unused void *pvParams);
void OLED_task(__unused void *pvParams);
void UI_task(__unused void *pvParams);

void oled_update(void);
void oled_setup(void);

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// GLOBALS
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// FreeRTOS
TaskHandle_t hLTCtimecode_task = NULL;
TaskHandle_t hOLED_task = NULL;
TaskHandle_t hUI_task = NULL;
SemaphoreHandle_t semNewFrame = NULL;
SemaphoreHandle_t sem1Hz = NULL;
SemaphoreHandle_t semOLED = NULL;

// Interrupts
static volatile bool tick_1Hz;                  // 1Hz Interrupt pin value;
static volatile uint16_t tick_count_32kHz;      // 32kHz RTC Interrupt counter
static volatile uint16_t tick_count_32kHz_prev; // Previous LTC bit output tick
portMUX_TYPE tick_32kHz_atomic = portMUX_INITIALIZER_UNLOCKED;

// --------------------------------------------
// RTC
// --------------------------------------------
static DS3231 RTCchip = DS3231();        // Direct access to DS3231
static DateTime RTCtime; 				 // The date and time storage class

// --------------------------------------------
// libLTC & Linear Timecode task variables
// --------------------------------------------
// Timecode OUT

static LTCEncoder *LTCencoder;
static enum LTC_TV_STANDARD tv;
static int LTCencoder_flags = 0;
static ltcsnd_sample_t *pLTCaudio_out;

static volatile uint16_t LTCfps  = LTC_FPS_MAX; // Timecode (24,30,60) default to 30fps tick
static volatile double   dLTCfps = LTC_FPS_F_MAX; // Timecode (24,30,60) default to 30fps tick
static volatile uint8_t  LTCframe = 0;          // Current frame (0..fps-1) from input
static volatile uint8_t  LTCbyte = 0;           // Current LTC bit from input
static volatile uint8_t  LTCbit = 0;            // Current LTC bit from input
static uint16_t constLTCbit_32kHzTicks;        // #ticks for one LTC bit
static uint16_t constLTCbyte_32kHzTicks;       // #ticks for one LTC byte
static uint16_t constLTCframe_32kHzTicks;      // #ticks for one LTC frame

// Timecode IN
static LTCDecoder *LTCdecoder;
static volatile double LTC_in_sample; // current ADC value (0..4095)

static int LTCaudiorate = LTC_AUDIO_RATE;
static SMPTETimecode SMPTEtime;
//static uint16_t ADCaudio_in[(LTC_AUDIO_RATE / LTC_FPS_MIN) + 1];
static ltcsnd_sample_t LTCaudio_in[(LTC_AUDIO_RATE / LTC_FPS_MIN) + 1];

// Graphics
#define OLED_BUFF_MAX 64
static char oled_buff[OLED_BUFF_MAX]; // yyyy:mm:dd:hh:mm:sec:ff

//static Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1);
static Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1 /*[optional] offscreen buffer*/);

// --------------------------------------------
// Forward declarations
// --------------------------------------------
void IRAM_ATTR interrupt_1Hz(void);
void IRAM_ATTR interrupt_32kHz(void);
double ReadVoltage(byte pin);

void sendNTPpacket(IPAddress &address); // NTP UDP Manual packet
bool startWPSPBC();                     // NO_WIFI_CHIPSET_HAS_EEPROM

#ifdef ENABLE_WIFI
// --------------------------------------------
// WIFI
// --------------------------------------------
//#define WIFI_CHIPSET_HAS_EEPROM
#ifndef WIFI_CHIPSET_HAS_EEPROM
#ifndef STASSID
#define STASSID "2924aspen"
#define STAPSK "deadpool"
#endif
const char *ssid = STASSID; // your network SSID (name)
const char *pass = STAPSK;  // your network password
#endif

// --------------------------------------------
// NTPClient (will be obsoleted with ezTime)
// --------------------------------------------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
#endif //ENABLE_WIFI

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// ARDUINO SETUP()
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Initialize Serial
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	Serial.begin(/*115200*/ 230400);
	delay(100);
	Serial.println();
	Serial.println("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-");
	Serial.println("-+-+-+-+-+-+-+-+-+-+ OpenTimecode   -+-+-+-+-+-+-+-+-+-+-+-+-");
	Serial.println("-+-+-+-+-+-+-+-+-+-+ ver 0.1 3/2021 -+-+-+-+-+-+-+-+-+-+-+-+-");
	Serial.println("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-");

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Initialize PINS
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	{
		// OUTPUTS
		SCOPESET(LED_BUILTIN);
		SCOPESET(LTC_OUTPUT_PIN); // Audio/LTC ADC Output

		// FYI Wire class below sets up SDA=GPIO21, SCK=GPIO22 (native ports)
		// FYI Wire class below sets up SDA1=GPIO19, SCK1=GPIO23 (gpio muxed)

		// DEBUG OUPUTS
		SCOPESET(SCOPE_INT_32KHZ);
		SCOPESET(SCOPE_TIMECODE_TASK);
#ifdef ESP32_PINS_SCOPE
		SCOPESET(SCOPE_TIMECODE_SAMPLE);
#endif
		SCOPESET(SCOPE_TIMECODE_FRAME);
		SCOPESET(SCOPE_DS3231);
		SCOPESET(SCOPE_OLED);
		//    SCOPESET(SCOPE_UNUSED1);
		//    SCOPESET(SCOPE_UNUSED2);

		// INPUTS
		pinMode(LTC_INPUT_PIN, INPUT); // Audio/LTC ADC Input
		pinMode(INT_32KHZ_PIN, INPUT); // 32kHz interrupt input
		pinMode(INT_1HZ_PIN, INPUT);   // 1Hz interrupt input
	}

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Initialize I2C
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	{
		Wire.begin(SDA,SCL,400*1000);		// RTC clock I2C bus @ 400k bps
	}

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Initialize frame tick variables
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	{
		constLTCframe_32kHzTicks = TICK_32kHz / LTCfps;
		constLTCbyte_32kHzTicks = TICK_32kHz / (LTCfps * LTC_PACKET_BYTES);
		constLTCbit_32kHzTicks = TICK_32kHz / (LTCfps * LTC_PACKET_BITS);
	}

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Initialize the RTC on Wire (bus 0)
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	{
		// Turn on the internal oscillator & 1Hz Square Wave but not when on Vbat
		RTCchip.enableOscillator(true, false, 0);
		// Turn on the 32kHz output
		RTCchip.enable32kHz(true);
		tick_count_32kHz = 0;
		tick_count_32kHz_prev = 0;
		// Note - we will synchronize the ticks when the interrupts are installed
	}

#if 0
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		// ADC
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		// https://github.com/espressif/arduino-esp32/issues/92#issuecomment-327476086
		// ADC_11db = 0-3.3v, ADC_6db = 0-2.2v, ADC_2.5db = 0-1.5v, 0db = 0-1v
	{
		// https://forum.arduino.cc/index.php?topic=580984.0
		analogReadResolution(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
		analogSetWidth(12);                         // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
		//  9-bit gives an ADC range of 0-511
		// 10-bit gives an ADC range of 0-1023
		// 11-bit gives an ADC range of 0-2047
		// 12-bit gives an ADC range of 0-4095
		analogSetClockDiv(1);                       // Set the divider for the ADC clock, default is 1, range is 1 - 255
		analogSetAttenuation(ADC_0db);              // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db

		// analogSetPinAttenuation(VP,ADC_11db);    // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
		// ADC_0db provides no attenuation so IN/OUT        = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
		// ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
		// ADC_6db provides an attenuation so that IN/OUT   = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
		// ADC_11db provides an attenuation so that IN/OUT  = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement
		//analogSetVRefPin(26);  // 25,26,27
		analogSetPinAttenuation(LTC_INPUT_PIN,ADC_2_5db); // ADC input range 0-1.5v

		// Join ADC1 to the LTC INPUT
		adcAttachPin(LTC_INPUT_PIN);
		//read a value
		LTC_in_sample = ReadVoltage(LTC_INPUT_PIN); // Not sure it this deals with !0db attenuation right
		Serial.print("Analog value = ");
		Serial.println(LTC_in_sample);
	}
#endif

	// ================================================================================
	// SETUP LTC ENCODER
	// ================================================================================
	{
		Serial.println("Starting LTC Encoder");

		// Audio sample/generated rate
		LTCaudiorate = TICK_8kHz; //TICK_32kHz; //44100, 48000, 96000;
		LTCfps = LTC_FPS_MAX;
		dLTCfps = LTC_FPS_F_MAX;

		// LTC TV Standard
		tv = LTC_TV_525_60; //<30fps
		//tv = LTC_TV_625_50;  //<25fps
		//tv = LTC_TV_1125_60; //<30fps
		//tv = LTC_TV_FILM_24; //<24fps

		// Flags of operation
		LTCencoder_flags |= LTC_USE_DATE;       //< LTCFrame <> SMPTEtime converter and LTCFrame increment/decrement use date, also set BGF2 to '1' when encoder is initialized or re-initialized (unless LTC_BGF_DONT_TOUCH is given)
		LTCencoder_flags |= LTC_TC_CLOCK;       //< the Timecode is wall-clock aka freerun. This also sets BGF1 (unless LTC_BGF_DONT_TOUCH is given)
		LTCencoder_flags |= LTC_BGF_DONT_TOUCH; //< encoder init or re-init does not touch the BGF bits (initial values after initialization is zero)
		//encoder_flags |= LTC_NO_PARITY;     //< parity bit is left untouched when setting or in/decrementing the encoder frame-number

		LTCencoder = ltc_encoder_create(LTCaudiorate,
																		dLTCfps,    // Framerate=30 (default & std max) (also support 24,25. No 60)
																		tv,                // TV Standard
																		LTCencoder_flags); // Config Flags
		pLTCaudio_out = (ltcsnd_sample_t *)calloc(ltc_encoder_get_buffersize(LTCencoder), sizeof(ltcsnd_sample_t));
		Serial.print("\tpLTCaudio_out buffersize = ");
		Serial.println(ltc_encoder_get_buffersize(LTCencoder));
	} // LTC ENCODER

	// ================================================================================
	// SETUP LTC DECODER
	// ================================================================================
	{
		Serial.println("Starting LTC Decoder");
		LTCdecoder = ltc_decoder_create(LTCaudiorate / LTCfps, // Sound buffer size (1 Frame @ 32.768kHz)
																		2);                    // #Queues
		memset(LTCaudio_in, 0, sizeof(LTCaudio_in));
	} // LTC DECODER

#ifdef ENABLE_WIFI
	// ================================================================================
	// Connect to WiFi, if the stored SSID is invalid join to a station using WPS button
	// ================================================================================
	{
		Serial.println("Starting WiFi");

		WiFi.mode(WIFI_STA);
#ifdef NO_WIFI_CHIPSET_HAS_EEPROM // No WiFi chipset embedded EEPROM storage of the prior SSID,PASS
		WiFi.begin(ssid, pass);
#else
		Serial.printf("Stored SSID='%s' PSK='%s'\n", WiFi.SSID().c_str(), WiFi.psk().c_str());
		WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str()); // reading data from EEPROM, last saved credentials
#endif
		int i = 30; // Wait for up to 30 seconds
		while (i--)
		{
			if (WiFi.status() != WL_CONNECTED)
			{
				delay(1000);
				Serial.print(".");
			}
			else
			{
				break;
			}
		}
		Serial.println();

		wl_status_t status = WiFi.status();
		if (status == WL_CONNECTED)
		{
			Serial.printf("\nConnected successfully to SSID '%s'\n", WiFi.SSID().c_str());
		}
		else
		{
			Serial.printf("\nCould not connect to WiFi. state='%d'\n", status);
			Serial.println("Please press WPS button on your router.\n Press any key to continue...");
			while (!Serial.available())
			{
				;
			}
			if (!startWPSPBC())
			{
				Serial.println("Failed to connect with WPS");
			}
		}
		Serial.print("Local IP = ");
		Serial.print(WiFi.localIP());
		Serial.print("/");
		Serial.println(WiFi.subnetMask());
	} // WiFi
#endif

#ifdef ENABLE_WIFI
	// ================================================================================
	// Get the TimeZone based on local public IP address
	// ================================================================================
	Serial.println("Starting Timezone Task");
	timezone_task();

	// ================================================================================
	// Get the Network time (placeholder while OLD menu system trigger to do so)
	// ================================================================================
	Serial.println("Starting Network Time Task");
	timeClient.begin();
	ntp_task();
#endif

	// Synchronize on RTC 'second' digit change
	DateTime oldtime = RTCchip.now();
	do
	{
		RTCtime = RTCchip.now();
	} while (oldtime.second() == RTCtime.second());
	// LTC input info
	LTCframe = 0;
	LTCbyte = 0;
	LTCbit = 0;
	tick_count_32kHz_prev = tick_count_32kHz = 0;
	// Initialize the LTC Encoder time from the RTC
	SMPTEtime.years = RTCtime.year();
	SMPTEtime.months = RTCtime.month();
	SMPTEtime.days = RTCtime.day();
	SMPTEtime.hours = RTCtime.hour();
	SMPTEtime.mins = RTCtime.minute();
	SMPTEtime.secs = RTCtime.second();
	SMPTEtime.frame = 0;
	ltc_encoder_set_timecode(LTCencoder, &SMPTEtime);

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Create the Interrupts AFTER Tasks & Semaphores they serve are created
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	/*
	semNewFrame = xSemaphoreCreateCounting(LTCfps,0);
	if(semNewFrame == NULL) {
			Serial.println("semNewFrame allocation failed");
			DEBUGFAIL(LED_BUILTIN);
	}
*/
#if 0
	sem1Hz = xSemaphoreCreateBinary();
	if (sem1Hz == NULL)
	{
		Serial.println("sem1Hz allocation failed");
		DEBUGFAIL(LED_BUILTIN);
	}

	semOLED = xSemaphoreCreateBinary();
	if (semOLED == NULL)
	{
		Serial.println("semOLED allocation failed");
		DEBUGFAIL(LED_BUILTIN);
	}
#endif

	// CORE 0 (primary core (default) ) Tasks
#if 0
	xTaskCreatePinnedToCore(UI_task,                  // Task function
							"UI",                     // Task name
							configMINIMAL_STACK_SIZE, // Stack size
							NULL,                     // Parameters
							1,                        // Priority (Idle Task is 0)
							&hUI_task,                // Returned Task Handle
							1);                       // core 0 (FreeRTOS) core 1(Ardiuno)
#endif

	xTaskCreatePinnedToCore(OLED_task,                    // Task function
							"OLED",                       // Task name
							2 * configMINIMAL_STACK_SIZE, // Stack size
							NULL,                         // Parameters
							100,                          // Priority (Idle Task is 0)
							&hOLED_task,                  // Returned Task Handle
							1);                           // core 0 (FreeRTOS) core 1(Ardiuno)

	// CORE 1 (second unused core) Tasks
	xTaskCreatePinnedToCore(LTCtimecode_task,         // Task function
							"Timecode",               // Task name
							2 * configMINIMAL_STACK_SIZE, // Stack size
							NULL,                     // Parameters
							/* !!! THIS NEEDS TO BE REALTIME PRIORITY !!! */
							254,                	// Priority (Idle Task is 0)
							&hLTCtimecode_task, 	// Returned Task Handle
							0);                 	// core 0 (FreeRTOS) core 1(Ardiuno)

	Serial.println("End Setup()");
	return;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// INTERRUPTS     (IRAM_ATTR puts it in internal ram which is faster)
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// 32kHz tick coming from RTC
void IRAM_ATTR interrupt_32kHz(void)
{
	BaseType_t taskYieldRequired = false;
	static uint16_t prev_tick = 0;
	SCOPEON(SCOPE_INT_32KHZ);
	tick_count_32kHz++;
	// Reduce 32kHz down to 8kHz to reduce compute
	// This will be replaced by a 4:1 frequency divider using external D flip-flops
	if ( 1 /*(tick_count_32kHz - prev_tick) > 3 */)
	{
		prev_tick = tick_count_32kHz;
		bool isTickOverflow = (tick_count_32kHz > TICK_32kHz);
		bool isNewLTCframe   = ( tick_count_32kHz - tick_count_32kHz_prev > constLTCframe_32kHzTicks );
		bool isNewLTCbyte = (tick_count_32kHz - tick_count_32kHz_prev > constLTCbyte_32kHzTicks);
		//bool isNewLTCbit   = ( tick_count_32kHz - tick_count_32kHz_prev > constLTCbit_32kHzTicks );

		//  Compare to the tick number for a given frame (0..29) at a given fps (24,25,30)
//		uint16_t lookuptick = trigger_ticks_32kHz[4 /*fps_to_trigger_ticks(dLTCfps)*/][LTCframe];
//		bool isLTCframeOverflow = (tick_count_32kHz > lookuptick);   // Late match
//		bool isNewLTCframe = (tick_count_32kHz == lookuptick);       // On time match
//		bool isLTCframeOverflow = (tick_count_32kHz > (lookuptick & ~0x3) ); // Late match
//		bool isNewLTCframe = (tick_count_32kHz == (lookuptick & ~0x3) );     // On time match

		if (isTickOverflow)
		{
			SCOPEON(LED_BUILTIN);
		} else {
			// LTCdecode audio (the buffer to decode)
			uint16_t audio_offset = (tick_count_32kHz % TICK_32kHz) % constLTCframe_32kHzTicks;
			audio_offset /= 4; // change 32kHz to 8kHz
			//Serial.println(audio_offset);
			LTCaudio_in[audio_offset] = digitalRead(LTC_INPUT_PIN) ? 218 : 38;

			// LTCencode
			uint8_t audio = pLTCaudio_out[audio_offset];
			//Serial.println(audio);
			// LTCencode audio (already created)
			// NOTE: The audio out buffer holds a single frame
			// Send next LTC frame DIGITAL audio value (unsigned char)
			/* 	-3.0 dBFS default
					e->enc_lo = 38;
					e->enc_hi = 218;
			*/
			if (audio > 128)
			{
				digitalWrite(LTC_OUTPUT_PIN, HIGH);
			}
			else
			{
				digitalWrite(LTC_OUTPUT_PIN, LOW);
			}
		}

		if (isNewLTCbyte)
		{
			++LTCbyte %= LTC_PACKET_BYTES;
			ltc_encoder_encode_byte(LTCencoder, LTCbyte, 1.0);
		}

		if (isNewLTCframe)
		{
			SCOPEON(SCOPE_TIMECODE_FRAME);
			/*
			snprintf(buff_debug,
						BUFF_DEBUG_SIZE,
						"%5d = trigger_ticks_32kHz[ fps_to_trigger_ticks[LTCfps=%d] ][LTCframe=%d]; isLTCframeOverflow=%c isNewLTCframe=%c\n\r",
						lookuptick,
						LTCfps,
						LTCframe,
						isLTCframeOverflow ? '1':'0',
						isNewLTCframe ? '1':'0');
			Serial.print(buff_debug);
		*/
			++LTCframe %= LTCfps;
			tick_count_32kHz %= TICK_32kHz;
			tick_count_32kHz_prev = tick_count_32kHz;

			ltc_encoder_inc_timecode(LTCencoder);
			ltc_encoder_get_timecode(LTCencoder, &SMPTEtime);

			// Wake up Timecode_task on each new frame
			//xSemaphoreGiveFromISR(semNewFrame,&taskYieldRequired);
			// https://exploreembedded.com/wiki/Resuming_Task_From_ISR
			taskYieldRequired = xTaskResumeFromISR(hLTCtimecode_task);
			if (isTickOverflow)
			{
				SCOPEOFF(LED_BUILTIN);
			}
			SCOPEOFF(SCOPE_TIMECODE_FRAME);
		}
	} //if( 8kHz )
	SCOPEOFF(SCOPE_INT_32KHZ);
	if (taskYieldRequired == true)
	{
		portYIELD_FROM_ISR();
	}
}

// 1Hz tick coming from RTC
void IRAM_ATTR interrupt_1Hz(void)
{
	BaseType_t taskYieldRequired = false;

	SCOPEON(SCOPE_INT_1HZ); // INTERRUPT PULSE

	//xSemaphoreGiveFromISR(sem1Hz,&taskYieldRequired);
	SCOPEOFF(SCOPE_INT_1HZ); // INTERRUPT PULSE
	if (taskYieldRequired)
	{
		portYIELD_FROM_ISR();
	}
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// TASKS
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void LTCtimecode_task(__unused void *pvParams) // Running on the 'other' core [0]
{
	TaskHandle_t taskself = xTaskGetCurrentTaskHandle();
#if 1
	Serial.print("Starting the LTCtimecode (");
	Serial.print((uint32_t)taskself);
	Serial.println(") task");
#endif
	//attachInterrupt(INT_1HZ_PIN, interrupt_1Hz, RISING);      // Make this BEFORE 32kHz
	attachInterrupt(INT_32KHZ_PIN, interrupt_32kHz, FALLING);   // Make this interrupt last

	for (;;)
	{
		// Wait for 32kHz interrupt to wake this task once per FRAME
		vTaskSuspend(hLTCtimecode_task);
		SCOPEON(SCOPE_TIMECODE_TASK);
		if (1 /* xSemaphoreTake(semNewFrame,portMAX_DELAY) */)
		{
			// Read current time from RTC on I2C bus 0
			if(SMPTEtime.secs == 0) {
				SCOPEON(SCOPE_DS3231);
				RTCtime = RTCchip.now();
				SCOPEOFF(SCOPE_DS3231);
			}
			else {
				RTCtime.setsecond(SMPTEtime.secs);
			}
			// Get updated audio out buffer data
			//int len = ltc_encoder_copy_buffer(LTCencoder, pLTCaudio_out);

			// Update the OLED display on each new Frame
			vTaskResume(hOLED_task);
		} // if semNewFrame
		SCOPEOFF(SCOPE_TIMECODE_TASK);
	} // for(;;)
}

void OLED_task(__unused void *pvParams)
{
	TaskHandle_t taskself = xTaskGetCurrentTaskHandle();
	Serial.print("Starting the OLED (");
	Serial.print((uint32_t)taskself);
	Serial.println(") task");

	Serial.println("OLED_task oled_setup()");
	oled_setup();

	for (;;)
	{
		vTaskSuspend(taskself);
		if (1 /*xSemaphoreTake(semOLED,portMAX_DELAY)*/)
		{
			SCOPEON(SCOPE_OLED);
			//Serial.println("OLED_task oled_update()");
			oled_update();
			SCOPEOFF(SCOPE_OLED);
		} // semOLED Take
	}   // for(;;)
}

void UI_task(__unused void *pvParams)
{
	TaskHandle_t taskself = xTaskGetCurrentTaskHandle();
	Serial.print("Starting the UI (");
	Serial.print((uint32_t)taskself);
	Serial.println(") task");

	for (;;)
	{
		vTaskSuspend(taskself);
	}
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// LOCAL SUB-ROUTINES
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Polynomial improved linear response in ADC reading based on 0-3.3v
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
double ReadVoltage(byte pin)
{
	double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
	if (reading < 1 || reading > 4095)
		return 0;
	// return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
	return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
	/* ADC readings v voltage
	*  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
	// Polynomial curve match, based on raw data thus:
	*   464     0.5
	*  1088     1.0
	*  1707     1.5
	*  2331     2.0
	*  2951     2.5 
	*  3775     3.0
	*/
} // Added an improved polynomial, use either, comment out as required

#ifdef ENABLE_WIFI
/* Wireless Protected Setup (WPS) join-wireless-network-by-a-button 
 *  https://gist.github.com/copa2/fcc718c6549721c210d614a325271389
 */
bool startWPSPBC()
{
	Serial.println("WPS config start");
	bool wpsSuccess = WiFi.beginWPSConfig();
	if (wpsSuccess)
	{
		// Well this means not always success :-/ in case of a timeout we have an empty ssid
		String newSSID = WiFi.SSID();
		if (newSSID.length() > 0)
		{
			// WPSConfig has already connected in STA mode successfully to the new station.
			Serial.printf("WPS finished. Connected successfully to SSID '%s'\n", newSSID.c_str());
		}
		else
		{
			wpsSuccess = false;
		}
	}
	return wpsSuccess;
}
#endif

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// ARDUINO LOOP() on Core 1
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void loop()
{
	// This has to run to reset the watchdog
}

void oled_setup() {
	Wire1.begin(SDA1,SCL1,400*1000);	// OLED I2C bus @ 4Mhz (100k std, 400k fast)
	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!oled.begin(SSD1306_SWITCHCAPVCC, 
						SCREEN_ADDRESS,
						false, 				// hard reset if there is a defined pin
						false) )			// run Wire1.begin()
	{
		Serial.println(F("SSD1306 allocation failed"));
		DEBUGFAIL(LED_BUILTIN);
	}

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Blank the display (the Adafruit logo is in the buffer at boot)
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	oled.clearDisplay();
	oled.display();

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Default OLED Text Formatting
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	oled.setTextSize(1);
	oled.setTextColor(SSD1306_WHITE);
	oled.setTextWrap(false);
	return;
}

void oled_update() {
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Update OLED Display
	oled.clearDisplay();	// memset the local buffer to zero

	// Let's make sure we're giving enough time to other tasks
	//taskYIELD();

	oled.setCursor(TIMECODE_X, TIMECODE_Y - 8);
	oled.setTextSize(1);
	snprintf(oled_buff, OLED_BUFF_MAX, "RTC %02d:%02d:%02d.%02d",
						RTCtime.hour(), RTCtime.minute(), RTCtime.second(), LTCframe);
	oled.println(oled_buff);

	//display.writeFillRect(0/*TIMECODE_X*/,TIMECODE_Y-8,
	//                      SCREEN_WIDTH,SCREEN_HEIGHT,
	//                      SSD1306_BLACK);
	oled.setCursor(TIMECODE_X, TIMECODE_Y);
	oled.setTextSize(1);
	// LTC Encode Timecode
	//      snprintf(oled_buff, OLED_BUFF_MAX,
	//                "%5d %02d:%02d:%02d.%02d", uxSemaphoreGetCount(semNewFrame),
	snprintf(oled_buff, OLED_BUFF_MAX, "LTC %02d:%02d:%02d.%02d",
						SMPTEtime.hours, SMPTEtime.mins, SMPTEtime.secs, SMPTEtime.frame);
	oled.println(oled_buff);

	// LTC Decode Timecode
//#warning Implement LTC Decode OLED code

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Print XXXXXXXXX
//#warning Implement other OLED code
	oled.setCursor(0, 64-8);
	static uint16_t oledcount = 0;
	oled.println(++oledcount);

	//oled.display();		// Regular Adafruit_SS1306::display()
	oled.fastdisplay(8);	// My faster implementation updating only changed pixels

	return;
}

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

// ESP32 Watchdog
#include <esp_task_wdt.h>

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
//#include "tcMenu_example_menu.h"
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
#include "8kHztick_table.h"
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
#define SCREEN_WIDTH 128	// OLED display width, in pixels
#define SCREEN_HEIGHT 64	// OLED display height, in pixels

// Timecode location (in pixels from top left) on the screen
#define TIMECODE_FONTSIZE 1 // 1 or 2
#define TIMECODE_X 12		// 60 = (SML_FONT_X*TIMECODE_INDENT)
#define TIMECODE_Y 16		// last line 56 // last-1 line 48 = (SML_FONT_Y*TIMECODE_LINE)

// RTC CLOCK
#define TICK_8KHZ 8192

// Linear Time Code
#define LTC_PACKET_BITS 80
#define LTC_PACKET_BYTES 10
#define LTC_FPS_F_MAX 30.0
#define LTC_FPS_MAX 30
#define LTC_FPS_F_MIN 24.0
#define LTC_FPS_MIN 24
#define LTC_AUDIO_RATE TICK_8KHZ

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// CLASSES & FORWARD DECLARATIONS
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void LTCtimecode_task(__unused void *pvParams);
void OLED_task(__unused void *pvParams);
void UI_task(__unused void *pvParams);

uint8_t readSerialTextBCD2(bool echo);

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
SemaphoreHandle_t xRTC = NULL;

// Interrupts
static volatile bool rtc_tick_1Hz;					// 1Hz Interrupt pin value;
static volatile uint16_t rtc_tick_count;			// 8kHz RTC Interrupt counter
static volatile uint16_t ltc_frame_tick_count;		// rtc_tick_count reset per frame
static volatile uint16_t ltc_frame_tick_count_prev; // the number of ticks in the prior frame

// --------------------------------------------
// RTC
// --------------------------------------------
static DS3231 RTCchip = DS3231(); // Direct access to DS3231
static DateTime RTCtime;		  // The date and time storage class
float RTCtemperature;

// --------------------------------------------
// libLTC & Linear Timecode task variables
// --------------------------------------------
// Timecode OUT

static LTCEncoder *LTCencoder;
static enum LTC_TV_STANDARD tv;
static int LTCencoder_flags = 0;
static ltcsnd_sample_t LTCaudio_out[(LTC_AUDIO_RATE / LTC_FPS_MIN) + 1];

static volatile uint16_t LTCfps = LTC_FPS_MAX;	// Timecode (24,30,60) default to 30fps tick
static volatile double dLTCfps = LTC_FPS_F_MAX; // Timecode (24,30,60) default to 30fps tick
static volatile uint8_t LTCframe = 0;			// Current frame (0..fps-1) from input
static volatile uint8_t LTCbyte = 0;			// Current LTC bit from input
static volatile uint8_t LTCbit = 0;				// Current LTC bit from input
static uint16_t constLTCbit_8kHzTicks;			// #ticks for one LTC bit
static uint16_t constLTCbyte_8kHzTicks;			// #ticks for one LTC byte
static uint16_t constLTCframe_8kHzTicks;		// #ticks for one LTC frame

// Timecode IN
static LTCDecoder *LTCdecoder;
static LTCFrameExt LTCinput;
static volatile double LTC_in_sample; // current ADC value (0..4095)

static int LTCaudiorate = LTC_AUDIO_RATE;
static SMPTETimecode SMPTEtime;
static SMPTETimecode SMPTEtimeIn;
//static uint16_t ADCaudio_in[(LTC_AUDIO_RATE / LTC_FPS_MIN) + 1];
static ltcsnd_sample_t LTCaudio_in[(LTC_AUDIO_RATE / LTC_FPS_MIN) + 1];
static ltcsnd_sample_t LTCaudio_in_copy[(LTC_AUDIO_RATE / LTC_FPS_MIN) + 1];

// Graphics
#define OLED_BUFF_MAX 64
static char oled_buff[OLED_BUFF_MAX]; // yyyy:mm:dd:hh:mm:sec:ff
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1);

// --------------------------------------------
// Forward declarations
// --------------------------------------------
void IRAM_ATTR interrupt_1Hz(void);
void IRAM_ATTR interrupt_rtc_clock(void);
double ReadVoltage(byte pin);

void sendNTPpacket(IPAddress &address); // NTP UDP Manual packet
bool startWPSPBC();						// NO_WIFI_CHIPSET_HAS_EEPROM

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
const char *pass = STAPSK;	// your network password
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
		SCOPESET(LTC_OUTPUT_PIN);	// Audio/LTC Output
		SCOPESET(LTC_INPUT_DECODE); // Audio/LTC Input sampled value

		// FYI Wire class below sets up SDA=GPIO21, SCK=GPIO22 (native ports)
		// FYI Wire class below sets up SDA1=GPIO19, SCK1=GPIO23 (gpio muxed)

		// DEBUG OUPUTS
		SCOPESET(SCOPE_INT_8KHZ);
		SCOPESET(SCOPE_TIMECODE_BYTE);
		SCOPESET(SCOPE_TIMECODE_FRAME);
		SCOPESET(SCOPE_TIMECODE_TASK);
		SCOPESET(SCOPE_DS3231);
		SCOPESET(SCOPE_OLED);

		// INPUTS
		pinMode(LTC_INPUT_PIN, INPUT); // Audio/LTC ADC Input
		pinMode(INT_8KHZ_PIN, INPUT);  // 8kHz interrupt input
									   //pinMode(INT_1HZ_PIN, INPUT);   // 1Hz interrupt input
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
		LTCaudiorate = TICK_8KHZ;
		LTCfps = 30;
		dLTCfps = 30.0;

		// LTC TV Standard
		tv = LTC_TV_525_60; //<30fps
		//tv = LTC_TV_625_50;  //<25fps
		//tv = LTC_TV_1125_60; //<30fps
		//tv = LTC_TV_FILM_24; //<24fps

		// Flags of operation
		LTCencoder_flags |= LTC_USE_DATE;		//< LTCFrame <> SMPTEtime converter and LTCFrame increment/decrement use date, also set BGF2 to '1' when encoder is initialized or re-initialized (unless LTC_BGF_DONT_TOUCH is given)
		LTCencoder_flags |= LTC_TC_CLOCK;		//< the Timecode is wall-clock aka freerun. This also sets BGF1 (unless LTC_BGF_DONT_TOUCH is given)
		LTCencoder_flags |= LTC_BGF_DONT_TOUCH; //< encoder init or re-init does not touch the BGF bits (initial values after initialization is zero)
		//encoder_flags |= LTC_NO_PARITY;     //< parity bit is left untouched when setting or in/decrementing the encoder frame-number

		LTCencoder = ltc_encoder_create((double)LTCaudiorate,
										dLTCfps,		   // Framerate=30 (default & std max) (also support 24,25. No 60)
										tv,				   // TV Standard
										LTCencoder_flags); // Config Flags
	}													   // LTC ENCODER

	// ================================================================================
	// SETUP LTC DECODER
	// ================================================================================
	{
		Serial.println("Starting LTC Decoder");
		LTCdecoder = ltc_decoder_create(LTCaudiorate / LTCfps, // Sound buffer size (1 Frame @ 8.192kHz)
										1);					   // #Queues
		memset(LTCaudio_in, 0, sizeof(LTCaudio_in));
		memset(LTCaudio_in_copy, 0, sizeof(LTCaudio_in_copy));
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

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// NOTE : Create Interrupts AFTER Tasks & Semaphores they serve are created
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	xRTC = xSemaphoreCreateMutex();
	if (xRTC == NULL)
	{
		Serial.println("xRTC mutex allocation failed");
		DEBUGFAIL(LED_BUILTIN);
	}

	// Create tasks in order of highest priority to lowest since
	// the scheduler is already running
	xTaskCreatePinnedToCore(LTCtimecode_task, // Task function
							"Timecode",		  // Task name
							2048,			  // Stack size
							NULL,			  // Parameters
							/* !!! THIS NEEDS TO BE REALTIME PRIORITY !!! */
							254,				// Priority (Idle Task is 0, OS tick is 1)
							&hLTCtimecode_task, // Returned Task Handle
							1);					// core 0 (FreeRTOS) core 1(Ardiuno)

	xTaskCreatePinnedToCore(OLED_task,	 // Task function
							"OLED",		 // Task name
							2048,		 // Stack size
							NULL,		 // Parameters
							100,		 // Priority (Idle Task is 0, OS tick is 1)
							&hOLED_task, // Returned Task Handle
							1);			 // core 0 (FreeRTOS) core 1(Ardiuno)
#if 0
	xTaskCreatePinnedToCore(UI_task,   // Task function
							"UI",	   // Task name
							4096,	   // Stack size
							NULL,	   // Parameters
							10,		   // Priority (Idle Task is 0, OS tick is 1)
							&hUI_task, // Returned Task Handle
							1);		   // core 0 (FreeRTOS) core 1(Ardiuno)
#endif

	esp_task_wdt_init(1, true); // Panic after 1 second

	Serial.println("End Setup()");
	return;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// INTERRUPTS     (IRAM_ATTR puts it in internal ram which is faster)
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

volatile uint32_t debug_trap = 0;
volatile bool interrupt_rtc_clock_active = false;
// 32kHz tick coming from RTC
void IRAM_ATTR interrupt_rtc_clock(void)
{
	SCOPEON(SCOPE_INT_8KHZ);
	if (interrupt_rtc_clock_active)
	{
		Serial.println("interrupt_rtc_clock - TRAP - interrupt re-entrance");
		SCOPEON(LED_BUILTIN);
	}
	interrupt_rtc_clock_active = true;

	BaseType_t taskYieldRequired = false;
	static uint16_t debug_rtc_tick_prev = 0;
	bool isTickOverflow = (rtc_tick_count > TICK_8KHZ);

	// THIS IS THE MASTER CLOCK 0..TICK_8KHZ-1
	rtc_tick_count++;
	ltc_frame_tick_count++;

#if 0
	// bug searching...
	// Did the tick somehow get more than 1 vs the previous?
	if (rtc_tick_count % TICK_8KHZ != (debug_rtc_tick_prev + 1) % TICK_8KHZ)
	{
		// Force a location for an software interrupt
		// with a volatile variable access
		debug_trap++;
		Serial.println("interrupt_rtc_clock - TRAP - unserviced interrupt");
		SCOPEON(LED_BUILTIN);
	}
	debug_rtc_tick_prev = rtc_tick_count;
#endif

	//  Compare to the tick number for a given frame (0..29) at a given fps (24,25,30)
	uint8_t trig_index = fps_to_trigger_ticks(dLTCfps);
	uint16_t tick_current_frame = trigger_ticks_8kHz[trig_index][LTCframe];

	if (isTickOverflow)
	{
		//SCOPEON(LED_BUILTIN);
	}
	else
	{
		// LTCdecode audio (the buffer to decode)
		//uint16_t audio_offset = rtc_tick_count % constLTCframe_8kHzTicks;
		bool value = digitalRead(LTC_INPUT_PIN);
		LTCaudio_in[ltc_frame_tick_count] = value ? 218 : 38;
		digitalWrite(LTC_INPUT_DECODE, value);

		// LTCencode
		uint8_t audio = LTCaudio_out[ltc_frame_tick_count];
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

	// Even with aggregated summing/multiplication errors doing #bytes * constLTCbyte_8kHzTicks
	// vs the actual tick it's supposed to be if you did the calc in floating point, it's close enough
	// and it saves having floating point math or a huge table
	// NOTE - If I see too much jitter on the LTC output I'll have to change this to floating point calcs.
	bool isNewLTCbyte = (rtc_tick_count > tick_current_frame + ((LTCbyte + 1) * constLTCbyte_8kHzTicks));
	if (isNewLTCbyte)
	{
		SCOPEON(SCOPE_TIMECODE_BYTE);
		++LTCbyte %= LTC_PACKET_BYTES;
		ltc_encoder_encode_byte(LTCencoder, LTCbyte, 1.0);

		uint16_t tick_next_frame = trigger_ticks_8kHz[trig_index][LTCframe + 1];
		bool isNewLTCframe = (rtc_tick_count > tick_next_frame);
		if (isNewLTCframe)
		{
			SCOPEON(SCOPE_TIMECODE_FRAME);
			/*
			snprintf(buff_debug,
						BUFF_DEBUG_SIZE,
						"%5d = trigger_ticks_8kHz[ fps_to_trigger_ticks[LTCfps=%d] ][LTCframe=%d]; isLTCframeOverflow=%c isNewLTCframe=%c\n\r",
						lookuptick,
						LTCfps,
						LTCframe,
						isLTCframeOverflow ? '1':'0',
						isNewLTCframe ? '1':'0');
			Serial.print(buff_debug);
		*/
			++LTCframe %= LTCfps;
			// Wrap to 0 when it is 8192 on the last frame of the second
			// Thus the 0'th tick is actually processed here at 8192
			// Do it in here because it only needs to be done when it's the last frame
			// Thus saving unnecessary processing in NewByte or NewSample
			rtc_tick_count %= TICK_8KHZ;
			ltc_frame_tick_count_prev = ltc_frame_tick_count;
			ltc_frame_tick_count = 0xFFFF;

			// Update LTC OUT clock
			ltc_encoder_inc_timecode(LTCencoder);
			// Get updated audio out buffer data
			ltc_encoder_copy_buffer(LTCencoder, LTCaudio_out);

			// Wake up Timecode_task on each new frame
			taskYieldRequired = xTaskResumeFromISR(hLTCtimecode_task);
			SCOPEOFF(SCOPE_TIMECODE_FRAME);
		} // if isNewLTCframe
		SCOPEOFF(SCOPE_TIMECODE_BYTE);
	} //if isNewLTCbyte

	if (isTickOverflow)
	{
		//SCOPEOFF(LED_BUILTIN);
	}

	SCOPEOFF(SCOPE_INT_8KHZ);
	interrupt_rtc_clock_active = false;

	if (taskYieldRequired == true)
	{
		portYIELD_FROM_ISR();
	}
}

// 1Hz tick coming from RTC
void IRAM_ATTR interrupt_1Hz(void)
{
	BaseType_t taskYieldRequired = false;

	//SCOPEON(SCOPE_INT_1HZ); // INTERRUPT PULSE

	//xSemaphoreGiveFromISR(sem1Hz,&taskYieldRequired);
	//SCOPEOFF(SCOPE_INT_1HZ); // INTERRUPT PULSE
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
void LTCtimecode_task(__unused void *pvParams)
{
	TaskHandle_t taskself = xTaskGetCurrentTaskHandle();
#if 1
	Serial.print("Starting the LTCtimecode (");
	Serial.print((uint32_t)taskself);
	Serial.println(") task");
#endif

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Initialize frame tick variables
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	{
		constLTCframe_8kHzTicks = TICK_8KHZ / LTCfps;
		constLTCbyte_8kHzTicks = TICK_8KHZ / (LTCfps * LTC_PACKET_BYTES);
		constLTCbit_8kHzTicks = TICK_8KHZ / (LTCfps * LTC_PACKET_BITS);
	}

	xSemaphoreTake(xRTC, portMAX_DELAY);
	{
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		// Initialize I2C
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		Wire.begin(SDA, SCL, 400 * 1000); // RTC clock I2C bus @ 400k bps

		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		// Initialize the RTC on Wire (bus 0)
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		// Turn on the internal oscillator & 8.192kHz Square Wave but not when on Vbat
		RTCchip.enableOscillator(true, false, 3);
		// Turn on the 32kHz output (this is divided using flip-flops to 8kHz)
		RTCchip.enable32kHz(false);

		rtc_tick_count = (uint16_t)0xFFFF; // This will trigger a new start in the interrupt
		ltc_frame_tick_count = (uint16_t)0xFFFF;

		RTCtemperature = RTCchip.getTemperature();

		// On power up it takes a while for the chip to be ready to read valid times
		//while (RTCchip.isBusy())
		//	;
#if 1
		RTCchip.setClockMode(false); // 12hr = true, 24hr = false
		RTCchip.setYear(0);
		RTCchip.setMonth(0);
		RTCchip.setDate(0);
		RTCchip.setHour(0);
		RTCchip.setMinute(0);
		RTCchip.setSecond(0);
		RTCtime = RTCchip.now();
#else
		// Synchronize on RTC 'second' digit change
		DateTime oldtime = RTCchip.now();
		do
		{
			RTCtime = RTCchip.now();
		} while (oldtime.second() == RTCtime.second());
		// Update all fields again now that we're at the start of a second
		RTCtime = RTCchip.now();
#endif
	}
	xSemaphoreGive(xRTC);

	// LTC input info
	LTCframe = 0;
	LTCbyte = 0;
	LTCbit = 0;
	// Initialize the LTC Encoder time from the RTC
	snprintf(SMPTEtime.timezone, 6, "+0000");
	SMPTEtime.years = RTCtime.year();
	SMPTEtime.months = RTCtime.month();
	SMPTEtime.days = RTCtime.day();
	SMPTEtime.hours = RTCtime.hour();
	SMPTEtime.mins = RTCtime.minute();
	SMPTEtime.secs = RTCtime.second();
	SMPTEtime.frame = LTCframe;
	ltc_encoder_set_timecode(LTCencoder, &SMPTEtime);
	ltc_encoder_encode_byte(LTCencoder, LTCbyte, 1.0);
	ltc_encoder_copy_buffer(LTCencoder, LTCaudio_out);

	//attachInterrupt(INT_1HZ_PIN, interrupt_1Hz, RISING);      // Make this BEFORE 32kHz
	attachInterrupt(INT_8KHZ_PIN, interrupt_rtc_clock, FALLING); // Make this interrupt last

	for (;;)
	{
		// Wait for RTC interrupt to wake this task once per FRAME
		vTaskSuspend(taskself);
		SCOPEON(SCOPE_TIMECODE_TASK);
		{
			esp_task_wdt_reset();
			/*
			uint8_t trig_index = fps_to_trigger_ticks(dLTCfps);
			uint16_t tick_current_frame = trigger_ticks_8kHz[trig_index][LTCframe];
			uint8_t LTCframe_prev = LTCframe - 1;
			if( LTCframe == 0 ) 
				LTCframe_prev = LTCfps;
			uint16_t tick_prev_frame = trigger_ticks_8kHz[trig_index][LTCframe_prev];
			size_t framesize = tick_current_frame - tick_prev_frame;
*/
			memcpy(LTCaudio_in_copy, LTCaudio_in, ltc_frame_tick_count_prev);
			// Decode the LTC input
			ltc_decoder_write(LTCdecoder,
							  LTCaudio_in_copy,
							  ltc_frame_tick_count_prev,
							  0);
			if (ltc_decoder_queue_length(LTCdecoder))
			{
				ltc_decoder_read(LTCdecoder, &LTCinput);
				//ltc_frame_to_time(&SMPTEtimeIn, &LTCinput.ltc, 1);
				SMPTEtimeIn.frame = LTCinput.ltc.frame_tens * 10 + LTCinput.ltc.frame_units;
				SMPTEtimeIn.secs = LTCinput.ltc.secs_tens * 10 + LTCinput.ltc.mins_units;
				SMPTEtimeIn.mins = LTCinput.ltc.mins_tens * 10 + LTCinput.ltc.secs_units;
				SMPTEtimeIn.hours = LTCinput.ltc.hours_tens * 10 + LTCinput.ltc.hours_units;
			}

			// Update the OLED display on each new Frame
			if (hOLED_task)
				vTaskResume(hOLED_task);
			// Update the UI
			if (hUI_task)
				vTaskResume(hUI_task);
		}
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
	//setupMenu();	// tcMenu system

	for (;;)
	{
		vTaskSuspend(taskself);
		SCOPEON(SCOPE_OLED);
		{
			ltc_encoder_get_timecode(LTCencoder, &SMPTEtime);

			SCOPEON(SCOPE_DS3231);
			xSemaphoreTake(xRTC, portMAX_DELAY);
			{
				RTCtime = RTCchip.now();
			}
			if (RTCtime.minute() == 0)
			{
				RTCtemperature = RTCchip.getTemperature();
			}
			xSemaphoreGive(xRTC);
			SCOPEOFF(SCOPE_DS3231);
			oled_update();
		}
		SCOPEOFF(SCOPE_OLED);
	} // for(;;)
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

void oled_setup()
{
	Wire1.begin(SDA1, SCL1, 400 * 1000); // OLED I2C bus @ 4Mhz (100k std, 400k fast)
	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!oled.begin(SSD1306_SWITCHCAPVCC,
					SCREEN_ADDRESS,
					false,	// hard reset if there is a defined pin
					false)) // run Wire1.begin()
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

void oled_update()
{
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Update OLED Display
	oled.clearDisplay(); // memset the local buffer to zero

	// RTC Time
	oled.setCursor(TIMECODE_X, TIMECODE_Y - 8);
	oled.setTextSize(1);
	snprintf(oled_buff, OLED_BUFF_MAX, "RTC CLK %02d:%02d:%02d",
			 RTCtime.hour(), RTCtime.minute(), RTCtime.second());
	oled.println(oled_buff);

	//display.writeFillRect(0/*TIMECODE_X*/,TIMECODE_Y-8,
	//                      SCREEN_WIDTH,SCREEN_HEIGHT,
	//                      SSD1306_BLACK);
	oled.setCursor(TIMECODE_X, TIMECODE_Y);
	oled.setTextSize(1);
	// LTC Encode Timecode
	snprintf(oled_buff, OLED_BUFF_MAX, "LTC OUT %02d:%02d:%02d.%02d",
			 SMPTEtime.hours, SMPTEtime.mins, SMPTEtime.secs, SMPTEtime.frame);
	oled.println(oled_buff);

	// LTC Decode Timecode
	//#warning Implement LTC Decode OLED code
	oled.setCursor(TIMECODE_X, TIMECODE_Y + 8);
	oled.setTextSize(1);
	snprintf(oled_buff, OLED_BUFF_MAX, "LTC  IN %02d:%02d:%02d.%02d",
			 SMPTEtimeIn.hours, SMPTEtimeIn.mins, SMPTEtimeIn.secs, SMPTEtimeIn.frame);
	oled.println(oled_buff);

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	// Print XXXXXXXXX
	//#warning Implement other OLED code
	oled.setCursor(0, 64 - 8);
	static uint16_t oledcount = 0;
	oled.println(++oledcount);

	//oled.display();		// Regular Adafruit_SS1306::display()
	oled.fastdisplay(8); // My faster implementation updating only changed pixels

	return;
}

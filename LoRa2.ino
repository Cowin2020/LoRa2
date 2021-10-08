/* *********************
LoRa sender and receiver
********************* */

/* Device Type */
#define DEVICE_SENDER 0  /* DEVICE_TYPE = 0 for sender */
#define DEVICE_RECEIVER 1  /* DEVICE_TYPE = 1 for receiver */
#define DEVICE_TYPE DEVICE_RECEIVER


/* Software parameters */
#define WIFI_SSID "Mark1"
#define WIFI_PASS "5654345234345"
#define HTTP_URL "http://cowin.hku.hk:8765/"

/* Features */
#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
//#define ENABLE_LORA_CALLBACK /* not working? */

/* Hardware parameters */
#define PIN_THERMOMETER 3
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define COM_BAUD 115200
#define IDLE_TIME 10000 /* milliseconds */

/* pin settings of TTGO LoRa32 OLED 2.1.6 */
#define PIN_LED         25
#define PIN_OLED_SDA    21
#define PIN_OLED_SCL    22
#define PIN_SDCARD_CS   13
#define PIN_SDCARD_MOSI 15
#define PIN_SDCARD_MISO  2
#define PIN_SDCARD_SCLK 14
#define PIN_LORA_MOSI   27
#define PIN_LORA_MISO   19
#define PIN_LORA_SCLK    5
#define PIN_LORA_CS     18
#define PIN_LORA_RST    23
#define PIN_LORA_DIO0   26
#define OLED_I2C_ADDR 0x3C
#define LORA_BAND 868E6

/* Helper macros */

#if DEVICE_TYPE != DEVICE_SENDER && DEVICE_TYPE != DEVICE_RECEIVER
	#error Unknown DEVICE_TYPE
#endif

#if defined(ENABLE_COM_OUTPUT) || defined(ENABLE_OLED_OUTPUT)
	#define ENABLE_OUTPUT
#endif

#if defined(ENABLE_LORA_CALLBACK) && defined(ENABLE_OLED_OUTPUT)
	#undef ENABLE_LORA_CALLBACK
#endif

#ifdef ENABLE_COM_OUTPUT
	#define Serial_print(x) Serial.print(x)
	#define Serial_println(x) Serial.println(x)
#else
	#define Serial_print(x) {}
	#define Serial_println(x) {}
#endif

#ifdef ENABLE_OLED_OUTPUT
	#define OLED_home() {OLED.clearDisplay();OLED.setCursor(0,0);}
	#define OLED_print(x) OLED.print(x)
	#define OLED_println(x) OLED.println(x)
	#define OLED_display() OLED.display()
#else
	#define OLED_home() {}
	#define OLED_print(x) {}
	#define OLED_println(x) {}
	#define OLED_display() {}
#endif

#define any_print(x) {Serial_print(x);OLED_print(x);}
#define any_println(x) {Serial_println(x);OLED_println(x);}

/* Includes */

#include <SPI.h>
#include <LoRa.h>

#ifdef ENABLE_OLED_OUTPUT
	#include <Wire.h>
	#include <Adafruit_SSD1306.h>
#endif

#if DEVICE_TYPE == DEVICE_SENDER
	#include <OneWire.h>
	#include <DallasTemperature.h>
#elif DEVICE_TYPE == DEVICE_RECEIVER
	#include <WiFi.h>
	#include <HTTPClient.h>
#endif

#include <SD_MMC.h>

/* Variables and Functions */

#ifdef ENABLE_OLED_OUTPUT
	static Adafruit_SSD1306 OLED(OLED_WIDTH, OLED_HEIGHT);
#endif

#if DEVICE_TYPE == DEVICE_SENDER
	static OneWire onewire_thermometer(PIN_THERMOMETER);
	static DallasTemperature thermometer(&onewire_thermometer);
#elif DEVICE_TYPE == DEVICE_RECEIVER
	static unsigned long int last_idle_millis;
	static int HTTP_status;
	static String LoRa_packet;
#endif

#if DEVICE_TYPE == DEVICE_RECEIVER
	void upload(void) {
		int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return;
		}
		HTTPClient HTTP_client;
		String const URL = String(HTTP_URL) + LoRa_packet;
		Serial_println(String("Upload to ") + URL);
		HTTP_client.begin(URL);
		HTTP_status = HTTP_client.GET();
		Serial_println(String("HTTP status: ") + String(HTTP_status));
	}

	String WiFi_status_message(int const WiFi_status) {
		switch (WiFi_status) {
		case WL_NO_SHIELD:
			return String("WiFi no shield");
		case WL_IDLE_STATUS:
			return String("WiFi idle");
		case WL_NO_SSID_AVAIL:
			return String("WiFi no SSID");
		case WL_SCAN_COMPLETED:
			return String("WiFi scan completed");
		case WL_CONNECTED:
			return String("WiFi connected");
		case WL_CONNECT_FAILED:
			return String("WiFi connect failed");
		case WL_CONNECTION_LOST:
			return String("WiFi connection lost");
		case WL_DISCONNECTED:
			return String("WiFi disconnected");
		default:
			return String("WiFi Status: ") + String(WiFi_status);
		}
	}

	#ifdef ENABLE_OLED_OUTPUT
		static void OLED_paint(void) {
			OLED_home();
			OLED_println(WiFi_status_message(WiFi.status()));
			OLED_print("HTTP: ");
			OLED_println(HTTP_status);
			OLED_print("LoRa ");
			OLED_print(LoRa_packet.length());
			OLED_print(": ");
			OLED_println(LoRa_packet);
			OLED_display();
		}
	#else
		#define OLED_paint() {}
	#endif

	static void on_idle(void) {
		unsigned long int const ms = millis();
		if (ms > last_idle_millis + IDLE_TIME) {
			last_idle_millis = ms;
			#ifdef ENABLE_OLED_OUTPUT
				OLED_paint();
			#endif
			Serial_println("SD Card type: " + String(SD_MMC.cardType()));
		}
	}

	static void LoRa_receive(int const packet_size) {
		if (packet_size) {
			LoRa_packet = "";
			for (size_t i=0; i<packet_size; ++i)
				LoRa_packet += char(LoRa.read());
			Serial_print("LoRa(");
			Serial_print(LoRa_packet.length());
			Serial_print("): ");
			Serial_println(LoRa_packet);
			OLED_paint();
			upload();
		}
		on_idle();
	}
#endif

static bool LoRa_available;

void setup() {
	/* initialize static variables */
	#if DEVICE_TYPE == DEVICE_RECEIVER
		last_idle_millis = millis();
		HTTP_status = 0;
		LoRa_packet = "";
	#endif

	/* initialize LED */
	#ifdef ENABLE_LED
		pinMode(PIN_LED, OUTPUT);
		digitalWrite(PIN_LED, LOW);
	#endif

	/* initialize serial port */
	#ifdef ENABLE_COM_OUTPUT
		Serial.begin(COM_BAUD);
	#endif

	/* initialize OLED */
	#ifdef ENABLE_OLED_OUTPUT
		Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
		OLED.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
		OLED.invertDisplay(false);
		OLED.setRotation(2);
		OLED.setTextSize(1);
		OLED.setTextColor(WHITE, BLACK);
		OLED.clearDisplay();
		OLED.setCursor(0, 0);
	#endif

	/* initialize WiFi */
	//WiFi.begin(WIFI_SSID, WIFI_PASS);

	/* Initialize LoRa */
	SPI.begin(PIN_LORA_SCLK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
	LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
	LoRa_available = LoRa.begin(LORA_BAND) == 1;
	if (LoRa_available) {
		#ifdef ENABLE_LORA_CALLBACK
			LoRa.onReceive(LoRa_receive);
			LoRa.receive();
		#endif
		any_println("LoRa initialized");
	} else {
		any_println("LoRa uninitialized");
	}
	OLED_println(WiFi_status_message(WiFi.status()));

	/* initialize SD card */
	if (SD_MMC.begin()) {
		any_println("SD card initialized");
	} else {
		any_println("SD card uninitialized");
	}

	OLED_display();
}

void loop() {
	if (!LoRa_available) {
		#ifdef ENABLE_LED
			digitalWrite(PIN_LED, HIGH);
			delay(100);
			digitalWrite(PIN_LED, LOW);
			delay(100);
		#endif
		return;
	}
	#if DEVICE_TYPE == DEVICE_SENDER
		OLED_home();
		thermometer.requestTemperatures();
		float const temperature = thermometer.getTempCByIndex(0);
		any_print("Temperature: ");
		any_println(temperature);
		LoRa.beginPacket();
		LoRa.print(temperature);
		LoRa.endPacket(false);
		OLED_display();
		return delay(10000);
	#elif DEVICE_TYPE == DEVICE_RECEIVER
		#ifndef ENABLE_LORA_CALLBACK
			LoRa_receive(LoRa.parsePacket());
		#endif
		on_idle();
	#else
		return delay(60000);
	#endif
}

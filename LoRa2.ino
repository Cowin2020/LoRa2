/* *********************
LoRa sender and receiver
********************* */

#include <Arduino.h>

#define DEVICE_ID 0
#define NUMBER_OF_SENDERS 1

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"
#define HTTP_URL "http://example.com/"
static char const SECRET_KEY[16] PROGMEM = "This is secret!";
static char const LOG_FILE_PATH[] PROGMEM = "/log.txt";
#define MEASURE_PERIOD 60000 /* milliseconds */
#define ACK_TIMEOUT 1000 /* milliseconds */
#define RESEND_TIMES 4

#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
#define ENABLE_SD_CARD

#define PIN_THERMOMETER 3
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define COM_BAUD 115200

#define PIN_LORA_RST 23  /* TTGO LoRa32 V2.1-1.6 version ? */
#define OLED_I2C_ADDR 0x3C
#define LORA_BAND 868E6

#include "config.h"

#include <SPI.h>
#include <LoRa.h>
#include <RNG.h>
#include <AES.h>
#include <GCM.h>

#define DEVICE_SENDER 0
#define DEVICE_RECEIVER 1
#if DEVICE_ID == 0
	#define DEVICE_TYPE DEVICE_RECEIVER
#else
	#define DEVICE_TYPE DEVICE_SENDER
#endif

#if defined(ENABLE_COM_OUTPUT) || defined(ENABLE_OLED_OUTPUT)
	#define ENABLE_OUTPUT
#endif

#if defined(ENABLE_LORA_CALLBACK) && defined(ENABLE_OLED_OUTPUT)
	#undef ENABLE_LORA_CALLBACK
#endif

#define PACKET_TIME   0
#define PACKET_ACK    1
#define PACKET_SEND   2
#define CIPHER_IV_LENGTH 12
#define CIPHER_TAG_SIZE 4

#ifdef ENABLE_OLED_OUTPUT
	#include <Wire.h>
	#include <Adafruit_SSD1306.h>
#endif

typedef unsigned long int Time;
typedef uint8_t Device;
typedef uint32_t SerialNumber;
typedef class GCM<AES128> AuthCipher;

struct PayloadSend {
	SerialNumber serial;
	float temperature;
};

class Schedule {
protected:
	bool enable;
	Time head;
	Time period;
public:
	Schedule(Time const initial_period) : enable(false), head(0), period(initial_period) {}
	void start(Time const now) {
		enable = true;
		head = now;
	}
	void stop(void) {
		enable = false;
	}
	virtual void run(Time const now) {
		head = now;
	}
	void tick(Time const now) {
		if (enable && (now < head || head+period <= now)) run(now);
	}
};

#ifdef ENABLE_COM_OUTPUT
	#define DEFINE_FUNCTION(TYPE) \
		static inline void Serial_print(TYPE x) { \
			Serial.print(x); \
		} \
		static inline void Serial_println(TYPE x) { \
			Serial.println(x); \
		}
	DEFINE_FUNCTION(char const *const)
	DEFINE_FUNCTION(String const &)
	DEFINE_FUNCTION(char const)
	DEFINE_FUNCTION(uint8_t const)
	DEFINE_FUNCTION(signed int const)
	DEFINE_FUNCTION(double const)
	#undef DEFINE_FUNCTION
#else
	#define DEFINE_FUNCTION(TYPE) \
		static inline void Serial_print(TYPE x) {} \
		static inline void Serial_println(TYPE x) {}
	DEFINE_FUNCTION(char const *const)
	DEFINE_FUNCTION(String const &)
	DEFINE_FUNCTION(char const)
	DEFINE_FUNCTION(uint8_t const)
	DEFINE_FUNCTION(signed int const)
	DEFINE_FUNCTION(double const)
	#undef DEFINE_FUNCTION
#endif

#ifdef ENABLE_OLED_OUTPUT
	static Adafruit_SSD1306 OLED(OLED_WIDTH, OLED_HEIGHT);
	static inline void OLED_home(void) {
		OLED.clearDisplay();
		OLED.setCursor(0,0);
	}
	#define DEFINE_FUNCTION(TYPE) \
		static inline void OLED_print(TYPE x) { \
			OLED.print(x); \
		} \
		static inline void OLED_println(TYPE x) { \
			OLED.println(x); \
		}
	DEFINE_FUNCTION(char const * const)
	DEFINE_FUNCTION(uint8_t const)
	DEFINE_FUNCTION(double const)
	#undef DEFINE_FUNCTION
	static inline void OLED_display(void) {
		OLED.display();
	}
#else
	static inline void OLED_home(void) {}
	#define DEFINE_FUNCTION(TYPE) \
		static inline void OLED_print(TYPE x) {} \
		static inline void OLED_println(TYPE x) {}
	DEFINE_FUNCTION(char const * const)
	DEFINE_FUNCTION(uint8_t const)
	DEFINE_FUNCTION(double const)
	#undef DEFINE_FUNCTION
	static inline void OLED_display(void) {}
#endif

#define DEFINE_FUNCTION(TYPE) \
	static inline void any_print(TYPE x) { \
		Serial_print(x); \
		OLED_print(x); \
	} \
	static inline void any_println(TYPE x) { \
		Serial_println(x); \
		OLED_println(x); \
	}
DEFINE_FUNCTION(char const *)
DEFINE_FUNCTION(uint8_t)
DEFINE_FUNCTION(double const)
#undef DEFINE_FUNCTION

static bool setup_error;

#if DEVICE_TYPE == DEVICE_SENDER
	#include <OneWire.h>
	#include <DallasTemperature.h>
	#ifdef ENABLE_SD_CARD
		#include <SD.h>
	#endif

	static class OneWire onewire_thermometer(PIN_THERMOMETER);
	static class DallasTemperature thermometer(&onewire_thermometer);
	static class SPIClass SPI_1(HSPI);

	static SerialNumber serial_current;
	static float temperature;
	#ifdef ENABLE_OLED_OUTPUT
		static char const *OLED_message;
	#endif
	static SerialNumber serial_wait;

	#ifdef ENABLE_SD_CARD
		static void dump_log_file(void) {
			class File file;
			signed int c;

			Serial_println("Log file BEGIN");
			file = SD.open(LOG_FILE_PATH, FILE_READ);
			for (;;) {
				c = file.read();
				if (c < 0) break;
				Serial_print(char(c));
			}
			Serial_println("");
			file.close();

			file = SD.open(LOG_FILE_PATH, FILE_WRITE);
			file.close();
			Serial_println("Log file END");
		}

		static void append_log_file(float const temperature) {
			class File file = SD.open(LOG_FILE_PATH, FILE_APPEND);
			if (!file) {
				any_println("Failed to append file log.txt");
			} else {
				if (!file.println(temperature)) {
					any_println("Failed to write file log.txt");
				} else {
					file.close();
				}
			}
		}
	#else
		static inline void append_log_file(float const temperature) {}
	#endif

	static void send_LoRa_serial(SerialNumber const serial) {
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_SEND));
		LoRa.write(uint8_t(DEVICE_ID));
		uint8_t nonce[CIPHER_IV_LENGTH];
		RNG.rand(nonce, sizeof nonce);
		LoRa.write(nonce, sizeof nonce);
		struct PayloadSend cleantext;
		cleantext.serial = serial;
		cleantext.temperature = temperature;
		AuthCipher cipher;
		if (!cipher.setKey((uint8_t const *)SECRET_KEY, sizeof SECRET_KEY)) {
			Serial_println("Unable to set key");
			#ifdef ENABLE_OLED_OUTPUT
				OLED_message = "Unable to set key";
			#endif
			return;
		}
		if (!cipher.setIV(nonce, sizeof nonce)) {
			Serial_println("Unable to set nonce");
			#ifdef ENABLE_OLED_OUTPUT
				OLED_message = "Unable to set nonce";
			#endif
			return;
		}
		struct PayloadSend ciphertext;
		cipher.encrypt((uint8_t *)&ciphertext, (uint8_t const *)&cleantext, sizeof cleantext);
		LoRa.write((uint8_t const *)&ciphertext, sizeof ciphertext);
		uint8_t tag[CIPHER_TAG_SIZE];
		cipher.computeTag(tag, sizeof tag);
		LoRa.write((uint8_t const *)&tag, sizeof tag);
		LoRa.endPacket(false);
	}

	static class Resend : public Schedule {
	protected:
		unsigned int counter;
	public:
		Resend(void) : Schedule(ACK_TIMEOUT) {}
		void start(Time const now) {
			if (!RESEND_TIMES) return;
			Schedule::start(now);
			counter = RESEND_TIMES;
		}
		virtual void run(Time const now) {
			Schedule::run(now);
			send_LoRa_serial(serial_wait);
			if (!--counter) stop();
		}
	} resend_schedule;

	static void send_LoRa(void) {
		send_LoRa_serial(serial_current);
		serial_wait = serial_current;
		++serial_current;
		resend_schedule.start(millis());
	}

	static class Measure : public Schedule {
	public:
		Measure(void) : Schedule(MEASURE_PERIOD) {}
		virtual void run(Time const now) {
			Schedule::run(now);
			OLED_home();
			thermometer.requestTemperatures();
			temperature = thermometer.getTempCByIndex(0);
			any_print("Temperature: ");
			any_println(temperature);
			#ifdef ENABLE_OLED_OUTPUT
				OLED_println(OLED_message);
			#endif
			send_LoRa();
			append_log_file(temperature);
			OLED_display();
		}
	} measure_schedule;

	static void LoRa_receive_ACK(void) {
		Device const device = LoRa.read();
		if (device != DEVICE_ID) return;
		uint8_t nonce[CIPHER_IV_LENGTH];
		if (LoRa.readBytes(nonce, sizeof nonce) != sizeof nonce) {
			Serial_println("LoRa ACK: fail to read cipher nonce");
			return;
		}
		SerialNumber ciphertext;
		if (LoRa.readBytes((char *)&ciphertext, sizeof ciphertext) != sizeof ciphertext) {
			Serial_println("LoRa ACK: fail to read serial number");
			return;
		}
		AuthCipher cipher;
		if (!cipher.setKey((uint8_t const *)SECRET_KEY, sizeof SECRET_KEY)) {
			Serial_println("LoRa ACK: fail to set cipher key");
			#ifdef ENABLE_OLED_OUTPUT
				OLED_message = "LoRa ACK: fail to set cipher key";
			#endif
		}
		if (!cipher.setIV(nonce, sizeof nonce)) {
			Serial_println("LoRa ACK: fail to set cipher nonce");
			#ifdef ENABLE_OLED_OUTPUT
				OLED_message = "LoRa ACK: fail to set cipher nonce";
			#endif
			return;
		}
		SerialNumber cleantext;
		cipher.decrypt((uint8_t *)&cleantext, (uint8_t const *)&ciphertext, sizeof cleantext);
		uint8_t tag[CIPHER_TAG_SIZE];
		if (LoRa.readBytes(tag, sizeof tag) != sizeof tag) {
			Serial_println("LoRa ACK: fail to read cipher tag");
			return;
		}
		if (!cipher.checkTag(tag, sizeof tag)) {
			Serial_println("LoRa ACK: invalid cipher tag");
			return;
		}
		if (cleantext != serial_wait) {
			Serial_println("LoRa ACK: serial number unmatched");
			return;
		}
		resend_schedule.stop();
	}

	static void LoRa_receive(signed int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		switch (packet_type) {
		case PACKET_ACK:
			if (packet_size != 1 + 1 + CIPHER_IV_LENGTH + sizeof (SerialNumber) + CIPHER_TAG_SIZE) {
				Serial_print("LoRa ACK: incorrect packet size: ");
				Serial_println(packet_size);
				break;
			}
			LoRa_receive_ACK();
			break;
		default:
			Serial_print("LoRa: incorrect packet type: ");
			Serial_println(packet_type);
		}

		/* add entropy to RNG */
		unsigned long int const microseconds = micros();
		RNG.stir((uint8_t const *)&microseconds, sizeof microseconds, 8);
	}

	void setup() {
		/* initialize internal states */
		setup_error = false;
		serial_current = 0;
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = "";
		#endif
		measure_schedule.start(0);
		RNG.begin("LoRa-2");

		/* initialize LED */
		#ifdef ENABLE_LED
			pinMode(LED_BUILTIN, OUTPUT);
			digitalWrite(LED_BUILTIN, LOW);
		#endif

		/* initialize serial port */
		#ifdef ENABLE_COM_OUTPUT
			Serial.begin(COM_BAUD);
		#endif

		/* initialize OLED */
		#ifdef ENABLE_OLED_OUTPUT
			Wire.begin(OLED_SDA, OLED_SCL);
			OLED.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
			OLED.invertDisplay(false);
			OLED.setRotation(2);
			OLED.setTextSize(1);
			OLED.setTextColor(WHITE, BLACK);
			OLED.clearDisplay();
			OLED.setCursor(0, 0);
		#endif

		/* Initialize thermometer */
		if (!setup_error) {
			thermometer.begin();
			DeviceAddress thermometer_address;
			if (thermometer.getAddress(thermometer_address, 0)) {
				any_println("Thermometer 0 found");
			} else {
				setup_error = true;
				any_println("Thermometer 0 not found");
			}
		}

		/* Initialize LoRa */
		if (!setup_error) {
			SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
			LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
			if (LoRa.begin(LORA_BAND) == 1) {
				any_println("LoRa initialized");
			} else {
				setup_error = true;
				any_println("LoRa uninitialized");
			}
		}

		/* initialize SD card */
		#ifdef ENABLE_SD_CARD
			if (!setup_error) {
				pinMode(SD_MISO, INPUT_PULLUP);
				SPI_1.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
				if (SD.begin(SD_CS, SPI_1)) {
					any_println("SD card initialized");
					Serial_println(String("SD Card type: ") + String(SD.cardType()));
					dump_log_file();
				} else {
					setup_error = true;
					any_println("SD card uninitialized");
				}
			}
		#endif

		OLED_display();
	}

	void loop() {
		if (setup_error) {
			#ifdef ENABLE_LED
				digitalWrite(LED_BUILTIN, HIGH);
				delay(100);
				digitalWrite(LED_BUILTIN, LOW);
				delay(100);
			#endif
			return;
		}
		unsigned long int const now = millis();
		measure_schedule.tick(now);
		resend_schedule.tick(now);
		LoRa_receive(LoRa.parsePacket());
		RNG.loop();
	}
#elif DEVICE_TYPE == DEVICE_RECEIVER
	#include <WiFi.h>
	#include <HTTPClient.h>

	static signed int HTTP_status;
	#ifdef ENABLE_OLED_OUTPUT
		static uint8_t OLED_device;
		static SerialNumber OLED_serial;
		static float OLED_temperature;
	#endif

	void upload_WiFi(Device const device, float const value) {
		signed int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return;
		}
		HTTPClient HTTP_client;
		String const URL = String(HTTP_URL) + String(device) + String("/") + String(value);
		Serial_println(String("Upload to ") + URL);
		HTTP_client.begin(URL);
		HTTP_status = HTTP_client.GET();
		Serial_println(String("HTTP status: ") + String(HTTP_status));
	}

	String WiFi_status_message(signed int const WiFi_status) {
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
			OLED_print("Device: ");
			OLED_println(OLED_device);
			OLED_print("Serial Number: ");
			OLED_println(OLED_serial);
			OLED_print("Temperature: ");
			OLED_println(OLED_temperature);
			OLED_display();
		}
	#else
		#define OLED_paint() {}
	#endif

	static void LoRa_send_ACK(Device const device, SerialNumber const serial) {
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_ACK));
		LoRa.write(uint8_t(device));
		uint8_t nonce[CIPHER_IV_LENGTH];
		RNG.rand(nonce, sizeof nonce);
		LoRa.write(nonce, sizeof nonce);
		SerialNumber const cleantext = serial;
		AuthCipher cipher;
		if (!cipher.setKey((uint8_t const *)SECRET_KEY, sizeof SECRET_KEY)) {
			Serial_println("LoRa SEND ACK: unable to set cipher key");
			return;
		}
		if (!cipher.setIV(nonce, sizeof nonce)) {
			Serial_println("LoRa SEND ACK: unable to set nonce");
			return;
		}
		SerialNumber ciphertext;
		cipher.encrypt((uint8_t *)&ciphertext, (uint8_t const *)&cleantext, sizeof cleantext);
		LoRa.write((uint8_t const *)&ciphertext, sizeof ciphertext);
		uint8_t tag[CIPHER_TAG_SIZE];
		cipher.computeTag(tag, sizeof tag);
		LoRa.write((uint8_t const *)&tag, sizeof tag);
		LoRa.endPacket(false);
	}

	static void LoRa_receive_SEND(void) {
		Device const device = LoRa.read();
		uint8_t nonce[CIPHER_IV_LENGTH];
		if (LoRa.readBytes(nonce, sizeof nonce) != sizeof nonce) {
			Serial_println("LoRa SEND: fail to read cipher nonce");
			return;
		}
		char ciphertext[sizeof (PayloadSend)];
		if (LoRa.readBytes((char *)&ciphertext, sizeof ciphertext) != sizeof ciphertext) {
			Serial_println("LoRa SEND: fail to read serial number");
			return;
		}
		AuthCipher cipher;
		if (!cipher.setKey((uint8_t const *)SECRET_KEY, sizeof SECRET_KEY)) {
			Serial_println("LoRa SEND: fail to set cipher key");
			return;
		}
		if (!cipher.setIV(nonce, sizeof nonce)) {
			Serial_println("LoRa SEND: fail to set cipher nonce");
			return;
		}
		struct PayloadSend cleantext;
		cipher.decrypt((uint8_t *)&cleantext, (uint8_t const *)&ciphertext, sizeof cleantext);
		uint8_t tag[CIPHER_TAG_SIZE];
		if (LoRa.readBytes(tag, sizeof tag) != sizeof tag) {
			Serial_println("LoRa SEND: fail to read cipher tag");
			return;
		}
		if (!cipher.checkTag(tag, sizeof tag)) {
			Serial_println("LoRa SEND: invalid cipher tag");
			return;
		}

		LoRa_send_ACK(device, cleantext.serial);

		/* TODO: actually use the values */
		#ifdef ENABLE_OLED_OUTPUT
			OLED_device = device;
			OLED_serial = cleantext.serial;
			OLED_temperature = cleantext.temperature;
		#endif

		OLED_paint();
		upload_WiFi(device, cleantext.temperature);
	}

	static void LoRa_receive(signed int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		struct {
			SerialNumber serial;
			float temperature;
		} ciphertext, cleantext;

		switch (packet_type) {
		case PACKET_SEND:
			if (packet_size != 1 + 1 + CIPHER_IV_LENGTH + sizeof (struct PayloadSend) + CIPHER_TAG_SIZE) {
				Serial_print("LoRa SEND: incorrect packet size");
				Serial_println(packet_size);
				break;
			}
			LoRa_receive_SEND();
			break;
		default:
			Serial_print("LoRa: incorrect packet type");
			Serial_println(packet_type);
		}

		/* add entropy to RNG */
		unsigned long int const microseconds = micros();
		RNG.stir((uint8_t const *)&microseconds, sizeof microseconds, 8);
	}

	void setup() {
		/* initialize static variables */
		HTTP_status = 0;

		/* initialize LED */
		#ifdef ENABLE_LED
			pinMode(LED_BUILTIN, OUTPUT);
			digitalWrite(LED_BUILTIN, LOW);
		#endif

		/* initialize serial port */
		#ifdef ENABLE_COM_OUTPUT
			Serial.begin(COM_BAUD);
		#endif

		/* initialize OLED */
		#ifdef ENABLE_OLED_OUTPUT
			Wire.begin(OLED_SDA, OLED_SCL);
			OLED.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
			OLED.invertDisplay(false);
			OLED.setRotation(2);
			OLED.setTextSize(1);
			OLED.setTextColor(WHITE, BLACK);
			OLED.clearDisplay();
			OLED.setCursor(0, 0);
		#endif

		/* Initialize LoRa */
		SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
		LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
		if (LoRa.begin(LORA_BAND) == 1) {
			#ifdef ENABLE_LORA_CALLBACK
				LoRa.onReceive(LoRa_receive);
				LoRa.receive();
			#endif
			any_println("LoRa initialized");
		} else {
			setup_error = true;
			any_println("LoRa uninitialized");
		}

		/* initialize WiFi */
		WiFi.begin(WIFI_SSID, WIFI_PASS);
		OLED_println(WiFi_status_message(WiFi.status()));

		OLED_display();
	}

	void loop() {
		if (setup_error) {
			#ifdef ENABLE_LED
				digitalWrite(LED_BUILTIN, HIGH);
				delay(100);
				digitalWrite(LED_BUILTIN, LOW);
				delay(100);
			#endif
			return;
		}
		#ifndef ENABLE_LORA_CALLBACK
			LoRa_receive(LoRa.parsePacket());
		#endif
	}
#endif

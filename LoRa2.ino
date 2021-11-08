/* *********************
LoRa sender and receiver
********************* */

#include <Arduino.h>

#define DEVICE_ID 0
#define NUMBER_OF_SENDERS 1

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"
#define HTTP_UPLOAD_FORMAT "http://www.example.com/%1$u/%2$lu/%3$F"
#define HTTP_UPLOAD_LENGTH 256
#define NTP_SERVER "stdtime.gov.hk"
#define SECRET_KEY "This is secret!"
#define LOG_FILE_PATH "/log.txt"
#define SYNCHONIZE_INTERVAL 7654321UL /* milliseconds */
#define MEASURE_INTERVAL 60000UL /* milliseconds */
#define ACK_TIMEOUT 1000UL /* milliseconds */
#define RESEND_TIMES 4

#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
#define ENABLE_SD_CARD
#define ENABLE_CLOCK

#define PIN_THERMOMETER 3
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define COM_BAUD 115200

#define PIN_LORA_RST 23  /* TTGO LoRa32 V2.1-1.6 version ? */
#define OLED_I2C_ADDR 0x3C
#define LORA_BAND 868000000

#include "config.h"

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

#define PACKET_TIME   0
#define PACKET_ACK    1
#define PACKET_SEND   2
#define CIPHER_IV_LENGTH 12
#define CIPHER_TAG_SIZE 4
#define DATETIME_SIZE 20

#include <stdlib.h>
#include <RNG.h>
#include <AES.h>
#include <GCM.h>
//	#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#ifdef ENABLE_OLED_OUTPUT
	#include <Adafruit_SSD1306.h>
#endif

#ifdef ENABLE_CLOCK
	#include <PCF85063TP.h>
#endif

typedef unsigned long int Time;
typedef uint8_t Device;
typedef uint32_t SerialNumber;
typedef class GCM<AES128> AuthCipher;

struct Payload_SEND {
	SerialNumber serial;
	float temperature;
};

#if 0
class DateTime {
protected:
	unsigned short int year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
	unsigned short int millisecond;
public:
	inline DateTime(
		unsigned short int initial_year,
		unsigned char initial_month,
		unsigned char initial_day,
		unsigned char initial_hour,
		unsigned char initial_minute,
		unsigned char initial_second = 0,
		unsigned short int initial_millisecond = 0
	) :
		year(initial_year),
		month(initial_month),
		day(initial_day),
		hour(initial_hour),
		minute(initial_minute),
		second(initial_second),
		millisecond(initial_millisecond)
	{}
	inline DateTime(void) : DateTime(0, 0, 0, 0, 0, 0, 0) {}
	DateTime(char const *const s) : millisecond(0) {
		unsigned int ye, mo, da, ho, mi, se, ms;
		if (sscanf(s, "%4u-%2u-%2uT%2u:%2u:%2uZ", &ye, &mo, &da, &ho, &mi, &se) != 6) {
			DateTime();
			return;
		}
		year = ye;
		month = mo;
		day = da;
		hour = ho;
		minute = mi;
		second = se;
	}
	bool leap_year(void) const {
		return !(year%400) || year%100 != 0 && !(year&3);
	}
	unsigned int days_of_month(void) const {
		switch (month) {
		case 1: return 31;
		case 2: return leap_year() ? 29 : 28;
		case 3: return 31;
		case 4: return 30;
		case 5: return 31;
		case 6: return 30;
		case 7: return 31;
		case 8: return 31;
		case 9: return 30;
		case 10: return 31;
		case 11: return 30;
		case 12: return 31;
		default: return 0;
		}
	}
	DateTime &operator+=(unsigned long int const ms) {
		div_t d;
		d = div(millisecond+ms, 1000);
		millisecond += d.rem;
		d = div(second+d.quot, 60);
		second += d.rem;
		d = div(minute+d.quot, 60);
		minute += d.rem;
		d = div(hour+d.quot, 24);
		hour += d.rem;
		day += d.quot;
		for (;;) {
			unsigned char const days = days_of_month();
			if (day <= days) break;
			day -= days;
			++month;
		}
		d = div(month-1, 12);
		month = d.rem+1;
		year += d.quot;
		return *this;
	}
	size_t string(char *const buffer, size_t const size) const {
		return snprintf(
			buffer, size,
			"%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
			year, month, day,
			hour, minute, second,
			millisecond
		);
	}
	String string(void) const {
		char buffer[24];
		(void)string(buffer, sizeof buffer);
		return String(buffer);
	}
};
#endif

class Schedule {
protected:
	bool enable;
	Time head;
	Time period;
	Time margin;
public:
	Schedule(Time const initial_period) : enable(false), head(0), period(initial_period), margin(0) {}
	void start(Time const now, Time const addition_period = 0) {
		enable = true;
		head = now;
		margin = addition_period;
	}
	void stop(void) {
		enable = false;
	}
	virtual void run(Time const now) {
		head = now;
	}
	void tick(Time const now) {
		if (enable && (now-head >= period+margin))
			run(now);
	}
};

#ifdef ENABLE_CLOCK
	static class PCD85063TP RTC;
#endif

static char const secret_key[16] PROGMEM = SECRET_KEY;
static char const log_file_path[] PROGMEM = LOG_FILE_PATH;

#ifdef ENABLE_COM_OUTPUT
	template <typename TYPE>
	inline void Serial_print(TYPE const x) {
			Serial.print(x);
	}
	template <typename TYPE>
	inline void Serial_println(TYPE x) {
		Serial.println(x);
	}
#else
	template <typename TYPE> inline void Serial_print(TYPE x) {}
	template <typename TYPE> inline void Serial_println(TYPE x) {}
#endif

#ifdef ENABLE_OLED_OUTPUT
	static Adafruit_SSD1306 OLED(OLED_WIDTH, OLED_HEIGHT);
	static String OLED_message;
	static void OLED_initialize(void) {
		//	Wire.begin(OLED_SDA, OLED_SCL);
		OLED.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
		OLED.invertDisplay(false);
		OLED.setRotation(2);
		OLED.setTextSize(1);
		OLED.setTextColor(WHITE, BLACK);
		OLED.clearDisplay();
		OLED.setCursor(0, 0);
	}
	inline static void OLED_home(void) {
		OLED.clearDisplay();
		OLED.setCursor(0,0);
	}
	template <typename TYPE>
	inline void OLED_print(TYPE x) {
		OLED.print(x);
	}
	template <typename TYPE>
	inline void OLED_println(TYPE x) {
		OLED.println(x);
	}
	inline static void OLED_display(void) {
		OLED.display();
	}
#else
	inline static void OLED_initialize(void) {}
	inline static void OLED_home(void) {}
	template <typename TYPE> inline void OLED_print(TYPE x) {}
	template <typename TYPE> inline void OLED_println(TYPE x) {}
	inline static void OLED_display(void) {}
#endif

template <typename TYPE>
inline void any_print(TYPE x) {
	Serial_print(x);
	OLED_print(x);
}
template <typename TYPE>
inline void any_println(TYPE x) {
	Serial_println(x);
	OLED_println(x);
}

#ifdef ENABLE_LED
	static void LED_initialize(void) {
		pinMode(LED_BUILTIN, OUTPUT);
		digitalWrite(LED_BUILTIN, LOW);
	}
	static void LED_flash(void) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(200);
		digitalWrite(LED_BUILTIN, LOW);
		delay(200);
	}
#else
	inline static void LED_initialize(void) {}
	inline static void LED_flash(void) {}
#endif

bool LoRa_initialize(void) {
	SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
	LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
	if (LoRa.begin(LORA_BAND) == 1) {
		any_println("LoRa initialized");
		return true;
	} else {
		any_println("LoRa uninitialized");
		return false;
	}
}

static bool LoRa_send_payload(char const *const message, void const *const payload, size_t const size) {
	uint8_t nonce[CIPHER_IV_LENGTH];
	RNG.rand(nonce, sizeof nonce);
	LoRa.write(nonce, sizeof nonce);
	AuthCipher cipher;
	if (!cipher.setKey((uint8_t const *)secret_key, sizeof secret_key)) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": unable to set key");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = "Unable to set key";
		#endif
		return false;
	}
	if (!cipher.setIV(nonce, sizeof nonce)) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": unable to set nonce");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = "Unable to set nonce";
		#endif
		return false;
	}
	char ciphertext[size];
	cipher.encrypt((uint8_t *)&ciphertext, (uint8_t const *)payload, size);
	LoRa.write((uint8_t const *)&ciphertext, sizeof ciphertext);
	uint8_t tag[CIPHER_TAG_SIZE];
	cipher.computeTag(tag, sizeof tag);
	LoRa.write((uint8_t const *)&tag, sizeof tag);
	return true;
}

static bool LoRa_receive_payload(char const *const message, void *const payload, size_t const size) {
	uint8_t nonce[CIPHER_IV_LENGTH];
	if (LoRa.readBytes(nonce, sizeof nonce) != sizeof nonce) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": fail to read cipher nonce");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = String("LoRa ") + message + ": fail to read cipher nonce";
		#endif
		return false;
	}
	char ciphertext[size];
	if (LoRa.readBytes(ciphertext, sizeof ciphertext) != size) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": fail to read time");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = String("LoRa ") + message + ": fail to read time";
		#endif
		return false;
	}
	uint8_t tag[CIPHER_TAG_SIZE];
	if (LoRa.readBytes(tag, sizeof tag) != sizeof tag) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": fail to read cipher tag");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = String("LoRa ") + message + ": fail to read cipher tag";
		#endif
		return false;
	}
	AuthCipher cipher;
	if (!cipher.setKey((uint8_t const *)secret_key, sizeof secret_key)) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": fail to set cipher key");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = String("LoRa ") + message + ": fail to set cipher key";
		#endif
		return false;
	}
	if (!cipher.setIV(nonce, sizeof nonce)) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": fail to set cipher nonce");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = String("LoRa ") + message + ": fail to set cipher nonce";
		#endif
		return false;
	}
	cipher.decrypt((uint8_t *)payload, (uint8_t const *)&ciphertext, size);
	if (!cipher.checkTag(tag, sizeof tag)) {
		Serial_print("LoRa ");
		Serial_print(message);
		Serial_println(": invalid cipher tag");
		#ifdef ENABLE_OLED_OUTPUT
			OLED_message = String("LoRa ") + message + ": invalid cipher tag";
		#endif
		return false;
	}
	return true;
}

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
	static SerialNumber serial_wait;
	static float temperature;

	#ifdef ENABLE_SD_CARD
		static void dump_log_file(void) {
			class File file;
			Serial_println("Log file BEGIN");
			file = SD.open(log_file_path, FILE_READ);
			for (;;) {
				signed int const c = file.read();
				if (c < 0) break;
				Serial_print(char(c));
			}
			Serial_println("");
			file.close();

			file = SD.open(log_file_path, FILE_WRITE);
			file.close();
			Serial_println("Log file END");
		}

		static void append_log_file(float const temperature) {
			class File file = SD.open(log_file_path, FILE_APPEND);
			if (!file) {
				any_println("Failed to append file log.txt");
			} else {
				file.printf(
					"%04u-%02u-%02uT%02u:%02u:%02uZ,%f\n",
					RTC.year, RTC.month, RTC.dayOfMonth,
					RTC.hour, RTC.minute, RTC.second
				);
				file.close();
			}
		}
	#else
		inline static void append_log_file(float const temperature) {}
	#endif

	static void LoRa_send_SEND(SerialNumber const serial) {
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_SEND));
		LoRa.write(uint8_t(DEVICE_ID));
		struct Payload_SEND const payload = {.serial = serial, .temperature = temperature};
		LoRa_send_payload("SEND", &payload, sizeof payload);
		LoRa.endPacket(true);
	}

	static class Resend : public Schedule {
	protected:
		unsigned int counter;
	public:
		Resend(void) : Schedule(ACK_TIMEOUT) {}
		void start(Time const now) {
			if (!RESEND_TIMES) return;
			counter = RESEND_TIMES;
			uint8_t margin;
			RNG.rand(&margin, sizeof margin);
			Schedule::start(now, margin & 0xFF);
		}
		virtual void run(Time const now) {
			Schedule::run(now);
			LoRa_send_SEND(serial_wait);
			if (!--counter) stop();
		}
	} resend_schedule;

	static void LoRa_send(void) {
		LoRa_send_SEND(serial_current);
		serial_wait = serial_current;
		if (serial_current & ~(~(SerialNumber)0 >> 1))
			serial_current = 0;
		else
			++serial_current;
		resend_schedule.start(millis());
	}

	static void LoRa_receive_TIME(void) {
		char datetime[DATETIME_SIZE+1];
		if (!LoRa_receive_payload("TIME", datetime, DATETIME_SIZE)) return;
		datetime[DATETIME_SIZE] = 0;
		unsigned int year, month, day, hour, minute, second;
		if (
			sscanf(
				datetime,
				"%4u-%2u-%2uT%2u:%2u:%2uZ",
				&year, &month, &day,
				&hour, &minute, &second
			) != 6
		) {
			Serial_print("LoRa Time: fail to parse time: ");
			Serial_println(datetime);
			return;
		}
		RTC.stopClock();
		RTC.fillByYMD(year, month, day);
		RTC.fillByHMS(hour, minute, second);
		RTC.setTime();
		RTC.startClock();
	}

	static void LoRa_receive_ACK(void) {
		Device const device = LoRa.read();
		if (device != DEVICE_ID) return;
		SerialNumber serial;
		if (!LoRa_receive_payload("ACK", &serial, sizeof serial)) return;
		if (serial != serial_wait) {
			Serial_println("LoRa ACK: serial number unmatched");
			return;
		}
		resend_schedule.stop();
	}

	static void LoRa_receive(signed int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		switch (packet_type) {
		case PACKET_TIME:
			if (packet_size != 1 + CIPHER_IV_LENGTH + DATETIME_SIZE + CIPHER_TAG_SIZE) {
				Serial_print("LoRa TIME: incorrect packet size: ");
				Serial_println(packet_size);
				break;
			}
			LoRa_receive_TIME();
			break;
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

	static class Measure : public Schedule {
	public:
		Measure(void) : Schedule(MEASURE_INTERVAL) {}
		virtual void run(Time const now) {
			Schedule::run(now);
			OLED_home();
			thermometer.requestTemperatures();
			temperature = thermometer.getTempCByIndex(0);
			any_print("Serial: ");
			any_println(serial_current);
			any_print("Temperature: ");
			any_println(temperature);
			#ifdef ENABLE_OLED_OUTPUT
				OLED_println(OLED_message);
			#endif
			append_log_file(temperature);
			LoRa_send();
			OLED_display();
		}
	} measure_schedule;

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
		LED_initialize();

		/* initialize serial port */
		#ifdef ENABLE_COM_OUTPUT
			Serial.begin(COM_BAUD);
		#else
			Serial.end();
		#endif

		/* initialize OLED */
		OLED_initialize();

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
		if (!setup_error)
			setup_error = !LoRa_initialize();

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

		/* initialize real-time clock */
		#ifdef ENABLE_CLOCK
			RTC.begin();
			RTC.startClock();
		#endif

		/* display setup result on OLED */
		OLED_display();
	}

	void loop() {
		if (setup_error) {
			LED_flash();
			return;
		}
		LoRa_receive(LoRa.parsePacket());
		measure_schedule.tick(millis());
		RNG.loop();
		LoRa_receive(LoRa.parsePacket());
		resend_schedule.tick(millis());
		RNG.loop();
	}
#elif DEVICE_TYPE == DEVICE_RECEIVER
	#include <WiFi.h>
	#include <WiFiUdp.h>
	#include <HTTPClient.h>
	#include <NTPClient.h>

	static SerialNumber serial_last[NUMBER_OF_SENDERS];
	static signed int HTTP_status;
	#ifdef ENABLE_OLED_OUTPUT
		static uint8_t OLED_device;
		static SerialNumber OLED_serial;
		static float OLED_temperature;
	#endif
	static WiFiUDP UDP;
	static NTPClient NTP(UDP, NTP_SERVER);

	static void upload_WiFi(Device const device, SerialNumber const serial, float const value) {
		signed int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return;
		}
		HTTPClient HTTP_client;
		char URL[HTTP_UPLOAD_LENGTH];
		snprintf(URL, sizeof URL, HTTP_UPLOAD_FORMAT, device, serial, value);
		Serial_print("Upload to ");
		Serial_println(URL);
		HTTP_client.begin(URL);
		HTTP_status = HTTP_client.GET();
		Serial_print("HTTP status: ");
		Serial_println(HTTP_status);
	}

	static String WiFi_status_message(signed int const WiFi_status) {
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
			OLED_println(OLED_message);
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
		if (!cipher.setKey((uint8_t const *)secret_key, sizeof secret_key)) {
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
		LoRa.endPacket(true);
	}

	static void LoRa_receive_SEND(void) {
		Device const device = LoRa.read();
		struct Payload_SEND payload;
		if (!LoRa_receive_payload("SEND", &payload, sizeof payload)) return;
		if (payload.serial < serial_last[device-1] && !(serial_last[device-1] & ~(~(SerialNumber)0 >> 1))) {
			Serial_println("LoRa SEND: serial number out of order");
			/* TODO: handle situation */
		}

		LoRa_send_ACK(device, payload.serial);
		serial_last[device-1] = payload.serial;
		#ifdef ENABLE_OLED_OUTPUT
			OLED_device = device;
			OLED_serial = payload.serial;
			OLED_temperature = payload.temperature;
		#endif

		OLED_paint();
		upload_WiFi(device, payload.serial, payload.temperature);
	}

	static void LoRa_receive(signed int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		switch (packet_type) {
		case PACKET_SEND:
			if (packet_size != 1 + 1 + CIPHER_IV_LENGTH + sizeof (struct Payload_SEND) + CIPHER_TAG_SIZE) {
				Serial_print("LoRa SEND: incorrect packet size: ");
				Serial_println(packet_size);
				break;
			}
			LoRa_receive_SEND();
			break;
		default:
			Serial_print("LoRa: incorrect packet type: ");
			Serial_println(packet_type);
		}

		/* add entropy to RNG */
		unsigned long int const microseconds = micros();
		RNG.stir((uint8_t const *)&microseconds, sizeof microseconds, 8);
	}

	class Synchronize : public Schedule {
	public:
		Synchronize(void) : Schedule(SYNCHONIZE_INTERVAL) {}
		virtual void run(Time const now) {
			Schedule::run(now);

			#ifdef ENABLE_CLOCK
				unsigned int const year = 2000+RTC.year;
				unsigned int const month = RTC.month;
				unsigned int const day = RTC.dayOfMonth;
				unsigned int const hour = RTC.hour;
				unsigned int const minute = RTC.minute;
				unsigned int const second = RTC.second;
			#else
				if (!NTP.isTimeSet()) return;
				time_t const epoch = NTP.getEpochTime();
				struct tm time;
				gmtime_r(&epoch, &time);
				unsigned int const year = 1900+time.tm_year;
				unsigned int const month = time.tm_mon;
				unsigned int const day = time.tm_mday;
				unsigned int const hour = time.tm_hour;
				unsigned int const minute = time.tm_min;
				unsigned int const second = time.tm_sec;
			#endif

			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_TIME));
			char payload[DATETIME_SIZE+1];
			RTC.getTime();
			snprintf(
				payload, sizeof payload,
				"%04u-%02u-%02uT%02u:%02u:%02uZ",
				year, month, day,
				hour, minute, second
			);
			LoRa_send_payload("TIME", payload, DATETIME_SIZE);
			LoRa.endPacket(true);
		}
	} synchronize_schedule;

	void setup() {
		/* initialize internal states */
		setup_error = false;
		HTTP_status = 0;
		for (size_t i=0; i<NUMBER_OF_SENDERS; ++i)
			serial_last[i] = 0;
		synchronize_schedule.start(0);

		/* initialize LED */
		LED_initialize();

		/* initialize serial port */
		#ifdef ENABLE_COM_OUTPUT
			Serial.begin(COM_BAUD);
		#else
			Serial.end();
		#endif

		/* initialize OLED */
		OLED_initialize();

		/* Initialize LoRa */
		if (!setup_error) {
			setup_error = !LoRa_initialize();
		}

		/* initialize WiFi */
		if (!setup_error) {
			WiFi.begin(WIFI_SSID, WIFI_PASS);
			//	any_println(WiFi_status_message(WiFi.status()));
		}

		/* initialize real-time clock */
		if (!setup_error) {
			RTC.begin();
			RTC.startClock();
			NTP.begin();
		}

		/* display setup result on OLED */
		OLED_display();
	}

	void loop() {
		if (setup_error) {
			LED_flash();
			return;
		}
		LoRa_receive(LoRa.parsePacket());
		if (WiFi.status() == WL_CONNECTED) {
			if (NTP.update()) {
				#ifdef ENABLE_CLOCK
					time_t const epoch = NTP.getEpochTime();
					struct tm time;
					gmtime_r(&epoch, &time);
					RTC.stopClock();
					RTC.fillByYMD(1900+time.tm_year, time.tm_mon+1, time.tm_mday);
					RTC.fillByHMS(time.tm_hour, time.tm_min, time.tm_sec);
					RTC.setTime();
					RTC.startClock();
				#endif
			}
		}
		synchronize_schedule.tick(millis());
		RNG.loop();
	}
#endif

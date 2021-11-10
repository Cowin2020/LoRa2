/* *********************
LoRa sender and receiver
********************* */

#include <Arduino.h>

#define DEVICE_ID 0
#define NUMBER_OF_SENDERS 1

#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
#define ENABLE_SD_CARD
#define ENABLE_CLOCK

#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"
#define HTTP_UPLOAD_FORMAT "http://www.example.com/%1$u/%2$lu/%3$s/%4$F"
#define HTTP_UPLOAD_LENGTH 256
#define NTP_SERVER "stdtime.gov.hk"
#define SECRET_KEY "This is secret!"
#define DATA_FILE_PATH "/data.csv"
#define CLEANUP_FILE_PATH "/cleanup.csv"
#define SYNCHONIZE_INTERVAL 7654321UL /* milliseconds */
#define RESEND_TIMES 4
#define ACK_TIMEOUT 1000UL /* milliseconds */
#define UPLOAD_INTERVAL 60000UL /* milliseconds */
#define MEASURE_INTERVAL 60000UL /* milliseconds */ /* MUST: > UPLOAD_INTERVAL */

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

#ifndef UPLOAD_INTERVAL
	#define UPLOAD_INTERVAL (ACK_TIMEOUT * (RESEND_TIMES + 2))
#endif

#define PACKET_TIME   0
#define PACKET_ACK    1
#define PACKET_SEND   2
#define CIPHER_IV_LENGTH 12
#define CIPHER_TAG_SIZE 4

#include <stdlib.h>
#include <time.h>
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
typedef GCM<AES128> AuthCipher;

struct DateTime {
	unsigned short int year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
};

struct Payload_SEND {
	SerialNumber serial;
	DateTime time;
	float temperature;
};

class Schedule {
protected:
	bool enable;
	Time head;
	Time period;
	Time margin;
public:
	Schedule(Time const initial_period) : enable(false), head(0), period(initial_period), margin(0) {}
	inline bool enabled(void) const {
		return enable;
	}
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
	static PCD85063TP RTC;
#endif

static char const secret_key[16] PROGMEM = SECRET_KEY;
static char const data_file_path[] PROGMEM = DATA_FILE_PATH;
static char const cleanup_file_path[] PROGMEM = CLEANUP_FILE_PATH;

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

static String String_from_DateTime(DateTime const *const datetime) {
	char buffer[48];
	snprintf(
		buffer, sizeof buffer,
		"%04u-%02u-%02uT%02u:%02u:%02uZ",
		datetime->year, datetime->month, datetime->day,
		datetime->hour, datetime->minute, datetime->second
	);
	return String(buffer);
}

#ifdef ENABLE_CLOCK
	static bool DateTime_now_available(void) {
		static bool available = false;
		if (available) return true;
		RTC.getTime();
		available =
			1 <= RTC.year       && RTC.year       <= 99 &&
			1 <= RTC.month      && RTC.month      <= 12 &&
			1 <= RTC.dayOfMonth && RTC.dayOfMonth <= 30 &&
			0 <= RTC.hour       && RTC.hour       <= 23 &&
			0 <= RTC.minute     && RTC.minute     <= 59 &&
			0 <= RTC.second     && RTC.second     <= 59;
		return available;
	}

	static DateTime DateTime_now(void) {
		RTC.getTime();
		return {
			.year = (unsigned short int)(2000U + RTC.year),
			.month = RTC.month,
			.day = RTC.dayOfMonth,
			.hour = RTC.hour,
			.minute = RTC.minute,
			.second = RTC.second
		};
	}
#endif

static bool setup_error;

#if DEVICE_TYPE == DEVICE_SENDER
	#include <OneWire.h>
	#include <DallasTemperature.h>
	#ifdef ENABLE_SD_CARD
		#include <SD.h>
	#endif

	#ifndef ENABLE_CLOCK
		#include <ESP32Time.h>
	#endif

	static OneWire onewire_thermometer(PIN_THERMOMETER);
	static DallasTemperature thermometer(&onewire_thermometer);
	static SPIClass SPI_1(HSPI);

	static SerialNumber serial_current;
	static Payload_SEND sending;
	static off_t wait_position;

	#ifndef ENABLE_CLOCK
		static bool clock_available;
		ESP32Time clock;

		static bool DateTime_now_available(void) {
			return clock_available;
		}

		static DateTime DateTime_now(void) {
			return {
				.year = (unsigned short int)clock.getYear(),
				.month = (unsigned char)clock.getMonth(),
				.day = (unsigned char)clock.getDay(),
				.hour = (unsigned char)clock.getHour(),
				.minute = (unsigned char)clock.getMinute(),
				.second = (unsigned char)clock.getSecond()
			};
		}
	#endif

	static void LoRa_send_SEND(void) {
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_SEND));
		LoRa.write(uint8_t(DEVICE_ID));
		LoRa_send_payload("SEND", &sending, sizeof sending);
		LoRa.endPacket(true);
	}

	static class Resend : public Schedule {
	protected:
		unsigned int counter;
	public:
		inline Resend(void) : Schedule(ACK_TIMEOUT) {}
		void start(Time const now) {
			if (!RESEND_TIMES) return;
			counter = RESEND_TIMES;
			uint8_t margin;
			RNG.rand(&margin, sizeof margin);
			Schedule::start(now, margin & 0xFF);
		}
		virtual void run(Time const now) {
			Schedule::run(now);
			LoRa_send_SEND();
			if (!--counter) stop();
		}
	} resend_schedule;

	static void LoRa_send(void) {
		LoRa_send_SEND();
		resend_schedule.start(millis());
	}

	static void LoRa_receive_TIME(void) {
		DateTime payload;
		if (!LoRa_receive_payload("TIME", &payload, sizeof payload)) return;

		#ifdef ENABLE_CLOCK
			RTC.stopClock();
			RTC.fillByYMD(payload.year, payload.month, payload.day);
			RTC.fillByHMS(payload.hour, payload.minute, payload.second);
			RTC.setTime();
			RTC.startClock();
		#else
			clock.setTime(
				payload.second, payload.minute, payload.hour,
				payload.day, payload.month, payload.year
			);
			clock_available = true;
		#endif
	}

	#ifdef ENABLE_SD_CARD
		static void dump_log_file(void) {
			Serial_println("Log file BEGIN");
			File file = SD.open(data_file_path, "r");
			for (;;) {
				signed int const c = file.read();
				if (c < 0) break;
				Serial_print(char(c));
			}
			Serial_println("");
			file.close();
			Serial_println("Log file END");
		}

		static void cleanup_data_file(void) {
			SD.remove(cleanup_file_path);
			if (!SD.rename(data_file_path, cleanup_file_path)) return;
			File cleanup_file = SD.open(cleanup_file_path, "r");
			if (!cleanup_file) {
				Serial_println("Fail to open clean-up file");
				return;
			}
			File data_file = SD.open(data_file_path, "w");
			if (!data_file) {
				Serial_println("Fail to create data file");
				cleanup_file.close();
				return;
			}
			for (;;) {
				String const line = cleanup_file.readStringUntil('\n');
				if (!line.length()) break;
				unsigned int sent;
				unsigned int year, month, day;
				unsigned int hour, minute, second;
				float temperature;
				if (
					sscanf(
						line.c_str(),
						"%1u,%4u-%2u-%2uT%2u:%2u:%2uZ,%f",
						&sent,
						&year, &month, &day,
						&hour, &minute, &second,
						&temperature
					) != 8
				) {
					Serial_print("Clean-up: invalid data: ");
					Serial_println(line);
					break;
				};
				if (sent) continue;
				data_file.printf(
					"0,%04u-%02u-%02uT%02u:%02u:%02uZ,%f\n",
					year, month, day,
					hour, minute, second,
					temperature
				);
			}
			cleanup_file.close();
			data_file.close();
			SD.remove(cleanup_file_path);
		}

		static void append_data_file(DateTime const *const time, float const temperature) {
			File file = SD.open(data_file_path, "a", true);
			if (!file) {
				any_println("Cannot append data file");
			} else {
				file.printf(
					"0,%04u-%02u-%02uT%02u:%02u:%02uZ,%f\n",
					time->year, time->month, time->day,
					time->hour, time->minute, time->second,
					temperature
				);
				file.close();
			}
		}

		static class Upload : public Schedule {
		protected:
			off_t position;
		public:
			inline Upload(void) : Schedule(UPLOAD_INTERVAL), position(0) {}
			void start(Time const now) {
				Schedule::start(now);
			}
			virtual void run(Time const now) {
				Schedule::run(now);
				File file = SD.open(DATA_FILE_PATH, "r");
				if (!file) {
					Serial_println("Upload: fail to open data file");
					return;
				}
				if (!file.seek(position)) {
					Serial_print("Upload: cannot seek: ");
					Serial_println(position);
					file.close();
					return;
				}
				off_t position_1;
				unsigned int sent;
				unsigned int year, month, day;
				unsigned int hour, minute, second;
				float temperature;
				for (;;) {
					String const line = file.readStringUntil('\n');
					position_1 = file.position();
					if (!line.length()) {
						/* end of file */
						file.close();
						return;
					}
					if (
						sscanf(
							line.c_str(),
							"%1u,%4u-%2u-%2uT%2u:%2u:%2uZ,%f",
							&sent,
							&year, &month, &day,
							&hour, &minute, &second,
							&temperature
						) != 8
					) {
						/* invalid data record */
						Serial_print("Upload: invalid data: ");
						Serial_println(line);
						position = position_1;
						file.close();
						return;
					};
					if (!sent) {
						/* send record to LoRa */
						sending.serial = serial_current++;
						sending.time.year = year;
						sending.time.month = month;
						sending.time.day = day;
						sending.time.hour = hour;
						sending.time.minute = minute;
						sending.time.second = second;
						sending.temperature = temperature;
						wait_position = position;
						file.close();
						LoRa_send();
					}
					position = position_1;
				}
			}
		} upload_schedule;
	#endif

	static class Measure : public Schedule {
	public:
		inline Measure(void) : Schedule(MEASURE_INTERVAL) {}
		virtual void run(Time const now) {
			Schedule::run(now);
			if (!DateTime_now_available()) return;

			OLED_home();
			DateTime time = DateTime_now();
			thermometer.requestTemperatures();
			float const temperature = thermometer.getTempCByIndex(0);
			Serial_print("Time: ");
			any_println(String_from_DateTime(&time));
			any_print("Temperature: ");
			any_println(temperature);
			#ifdef ENABLE_OLED_OUTPUT
				OLED_println(OLED_message);
				OLED_message = "";
			#endif
			#ifdef ENABLE_SD_CARD
				append_data_file(&time, temperature);
			#else
				sending.serial = serial_current++;
				sending.time = time;
				sending.temperature = thermometer;
				LoRa_send();
			#endif
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
		upload_schedule.start(0);
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
					cleanup_data_file();
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
		#else
			clock_available = false;
		#endif

		/* display setup result on OLED */
		OLED_display();
	}

	static void LoRa_receive_ACK(void) {
		Device const device = LoRa.read();
		if (device != DEVICE_ID) return;
		SerialNumber serial;
		if (!LoRa_receive_payload("ACK", &serial, sizeof serial)) return;
		if (serial != sending.serial) {
			Serial_println("LoRa ACK: serial number unmatched");
			return;
		}
		resend_schedule.stop();

		#ifdef ENABLE_SD_CARD
			File file = SD.open(data_file_path, "r+");
			if (!file) {
				Serial_println("LoRa ACK: fail to open data file");
				return;
			}
			if (!file.seek(wait_position)) {
				Serial_print("LoRa ACK: fail to seek data file: ");
				Serial_println(wait_position);
				return;
			}
			file.write('1');
			file.close();

			upload_schedule.start(millis());
		#endif
	}

	static void LoRa_receive(signed int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		switch (packet_type) {
		case PACKET_TIME:
			if (packet_size != 1 + CIPHER_IV_LENGTH + sizeof (DateTime) + CIPHER_TAG_SIZE) {
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
		LoRa_receive(LoRa.parsePacket());
		upload_schedule.tick(millis());
		RNG.loop();
	}
#elif DEVICE_TYPE == DEVICE_RECEIVER
	#include <WiFi.h>
	#include <WiFiUdp.h>
	#include <HTTPClient.h>
	#include <NTPClient.h>

	static SerialNumber serial_last[NUMBER_OF_SENDERS];
	static signed int HTTP_status;
	static WiFiUDP UDP;
	static NTPClient NTP(UDP, NTP_SERVER);

	#ifndef ENABLE_CLOCK
		inline static bool DateTime_now_available(void) {
			return NTP.isTimeSet();
		}
		static DateTime DateTime_now(void) {
			time_t const epoch = NTP.getEpochTime();
			struct tm time;
			gmtime_r(&epoch, &time);
			return {
				.year = (unsigned short int)(1900 + time.tm_year),
				.month = (unsigned char)(time.tm_mon + 1),
				.day = (unsigned char)time.tm_mday,
				.hour = (unsigned char)time.tm_hour,
				.minute = (unsigned char)time.tm_min,
				.second = (unsigned char)time.tm_sec
			};
		}
	#endif

	static bool WiFi_upload(Device const device, SerialNumber const serial, char const *const time, float const value) {
		signed int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return false;
		}
		HTTPClient HTTP_client;
		char URL[HTTP_UPLOAD_LENGTH];
		snprintf(URL, sizeof URL, HTTP_UPLOAD_FORMAT, device, serial, time, value);
		Serial_print("Upload to ");
		Serial_println(URL);
		HTTP_client.begin(URL);
		HTTP_status = HTTP_client.GET();
		Serial_print("HTTP status: ");
		Serial_println(HTTP_status);
		if (HTTP_status != 200) return false;
		return true;
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

	static void LoRa_send_ACK(Device const device, SerialNumber const serial) {
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_ACK));
		LoRa.write(uint8_t(device));
		LoRa_send_payload("ACK", &serial, sizeof serial);
		LoRa.endPacket(true);
	}

	static void LoRa_receive_SEND(void) {
		Device const device = LoRa.read();
		Payload_SEND payload;
		if (!LoRa_receive_payload("SEND", &payload, sizeof payload)) return;
		if (payload.serial < serial_last[device-1] && !(serial_last[device-1] & ~(~(SerialNumber)0 >> 1)))
			Serial_println("LoRa SEND: serial number out of order");
		char time[48];
		snprintf(
			time, sizeof time,
			"%04u-%02u-%02uT%02u:%02u:%02uZ",
			payload.time.year, payload.time.month, payload.time.day,
			payload.time.hour, payload.time.minute, payload.time.second
		);
		serial_last[device-1] = payload.serial;
		#ifdef ENABLE_OLED_OUTPUT
			OLED_home();
			OLED_println(WiFi_status_message(WiFi.status()));
			OLED_print("HTTP: ");
			OLED_println(HTTP_status);
			OLED_print("Device ");
			OLED_print(device);
			OLED_print(" Serial ");
			OLED_println(payload.serial);
			OLED_println(time);
			OLED_print("Temperature: ");
			OLED_println(payload.temperature);
			OLED_println(OLED_message);
			OLED_message = "";
			OLED_display();
		#endif
		if (!WiFi_upload(device, payload.serial, time, payload.temperature)) return;
		LoRa_send_ACK(device, payload.serial);
	}

	static void LoRa_receive(signed int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		switch (packet_type) {
		case PACKET_SEND:
			if (packet_size != 1 + 1 + CIPHER_IV_LENGTH + sizeof (Payload_SEND) + CIPHER_TAG_SIZE) {
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

	static class Synchronize : public Schedule {
	public:
		Synchronize(void) : Schedule(SYNCHONIZE_INTERVAL) {}
		virtual void run(Time const now) {
			Schedule::run(now);
			if (!DateTime_now_available()) return;
			DateTime const payload = DateTime_now();

			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_TIME));
			LoRa_send_payload("TIME", &payload, sizeof payload);
			LoRa.endPacket(true);
		}
	} synchronize_schedule;

	void setup() {
		/* initialize internal states */
		setup_error = false;
		HTTP_status = 0;
		for (size_t i = 0; i < NUMBER_OF_SENDERS; ++i)
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
		#ifdef ENABLE_CLOCK
			if (!setup_error) {
				RTC.begin();
				RTC.startClock();
				NTP.begin();
				NTP.setUpdateInterval(SYNCHONIZE_INTERVAL);
			}
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

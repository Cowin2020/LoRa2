/* *********************
LoRa sender and receiver
********************* */

#include <Arduino.h>


/* Network ID */
#define DEVICE_ID 0
#define NUMBER_OF_SENDERS 1


/* Features */
#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
#define ENABLE_CLOCK
#define ENABLE_SD_CARD
#define ENABLE_DALLAS
#define ENABLE_BME
#define ENABLE_LTR


/* For Debug */
//	#define DEBUG_CLEAN_OLD_DATA


/* Software Parameters */
#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"
#define HTTP_UPLOAD_FORMAT "http://www.example.com/%1$u/%2$lu/%3$s/%4$F"
#define HTTP_UPLOAD_LENGTH 256
#define HTTP_AUTHORIZATION_TYPE ""
#define HTTP_AUTHORIZATION_CODE ""
#define NTP_SERVER "stdtime.gov.hk"
#define SECRET_KEY "This is secret!"
#define DATA_FILE_PATH "/data.csv"
#define CLEANUP_FILE_PATH "/cleanup.csv"
#define SYNCHONIZE_INTERVAL 7654321UL /* milliseconds */
#define RESEND_TIMES 4
#define ACK_TIMEOUT 1000UL /* milliseconds */
#define UPLOAD_INTERVAL 6000UL /* milliseconds */
#define MEASURE_INTERVAL 60000UL /* milliseconds */ /* MUST: > UPLOAD_INTERVAL */


/* Hardware Parameters */
#define COM_BAUD 115200
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_I2C_ADDR 0x3C
#define DALLAS_PIN 3
#define LORA_PIN_RST 23  /* TTGO LoRa32 V2.1-1.6 version ? */
#define LORA_BAND 868000000


/* ************************************************************************** */

#define DEVICE_SENDER 0
#define DEVICE_RECEIVER 1

#include "config.h"

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

#define PACKET_TIME 0
#define PACKET_ACK  1
#define PACKET_SEND 2
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

typedef unsigned long int Time;
typedef uint8_t Device;
typedef uint32_t SerialNumber;
typedef GCM<AES128> AuthCipher;

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

	template <typename TYPE>
	inline void Serial_println(TYPE const x, int option) {
		Serial.println(x, option);
	}
#else
	template <typename TYPE> inline void Serial_print(TYPE x) {}
	template <typename TYPE> inline void Serial_println(TYPE x) {}
#endif

#ifdef ENABLE_OLED_OUTPUT
	static Adafruit_SSD1306 OLED(OLED_WIDTH, OLED_HEIGHT);
	static class String OLED_message;

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

	template <typename TYPE>
	inline void OLED_println(TYPE const x, int option) {
		OLED.println(x, option);
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

template <typename TYPE>
inline void any_println(TYPE x, int option) {
	Serial_println(x, option);
	OLED_println(x, option);
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

struct DateTime {
	unsigned short int year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
};

static class String String_from_DateTime(struct DateTime const *const datetime) {
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
	#include <PCF85063TP.h>
	namespace RTC {
		class PCD85063TP external_clock;

		static void initialize(void) {
			external_clock.begin();
			external_clock.startClock();
		}

		static void set(struct DateTime const *const datetime) {
			external_clock.stopClock();
			external_clock.fillByYMD(datetime->year, datetime->month, datetime->day);
			external_clock.fillByHMS(datetime->hour, datetime->minute, datetime->second);
			external_clock.setTime();
			external_clock.startClock();
		}

		static bool ready(void) {
			static bool available = false;
			if (available) return true;
			external_clock.getTime();
			available =
				1 <= external_clock.year       && external_clock.year       <= 99 &&
				1 <= external_clock.month      && external_clock.month      <= 12 &&
				1 <= external_clock.dayOfMonth && external_clock.dayOfMonth <= 30 &&
				0 <= external_clock.hour       && external_clock.hour       <= 23 &&
				0 <= external_clock.minute     && external_clock.minute     <= 59 &&
				0 <= external_clock.second     && external_clock.second     <= 59;
			return available;
		}

		static struct DateTime now(void) {
			external_clock.getTime();
			return {
				.year = (unsigned short int)(2000U + external_clock.year),
				.month = external_clock.month,
				.day = external_clock.dayOfMonth,
				.hour = external_clock.hour,
				.minute = external_clock.minute,
				.second = external_clock.second
			};
		}
	}
#endif

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

namespace LORA {
	static bool initialize(void) {
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

	static bool send_payload(char const *const message, void const *const payload, size_t const size) {
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

	static bool receive_payload(char const *const message, void *const payload, size_t const size) {
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
}

struct Data {
	#ifdef ENABLE_DALLAS
		float dallas_temperature;
	#endif
	#ifdef ENABLE_BME
		float bme_temperature;
		float bme_pressure;
		float bme_humidity;
	#endif
	#ifdef ENABLE_LTR
		float ltr_ultraviolet;
	#endif
	struct DateTime time;
};
//	__attribute__((packed));

struct Payload_SEND {
	SerialNumber serial;
	struct Data data;
};
//	__attribute__((packed));

static bool setup_error;

/* ************************************************************************** */

#if DEVICE_TYPE == DEVICE_SENDER
	#ifdef ENABLE_SD_CARD
		#include <SD.h>

		static class SPIClass SPI_1(HSPI);
	#endif

	#ifdef ENABLE_DALLAS
		#include <OneWire.h>
		#include <DallasTemperature.h>

		static class OneWire onewire_thermometer(DALLAS_PIN);
		static class DallasTemperature dallas(&onewire_thermometer);
	#endif

	#ifdef ENABLE_BME
		#include <Adafruit_Sensor.h>
		#include <Adafruit_BME280.h>

		static class Adafruit_BME280 BME;
	#endif

	#ifdef ENABLE_LTR
		#include <Adafruit_LTR390.h>

		static class Adafruit_LTR390 LTR;
	#endif

	#ifndef ENABLE_CLOCK
		#include <ESP32Time.h>
		namespace RTC {
			static bool clock_available;
			static class ESP32Time esp32time;

			static void initialize(void) {
				clock_available = false;
			}
			
			static void set(struct DateTime const *const datetime) {
				esp32time.setTime(
					datetime->second, datetime->minute, datetime->hour,
					datetime->day, datetime->month, datetime->year
				);
				clock_available = true;
			}

			static bool ready(void) {
				return clock_available;
			}

			static struct DateTime now(void) {
				return {
					.year = (unsigned short int)esp32time.getYear(),
					.month = (unsigned char)esp32time.getMonth(),
					.day = (unsigned char)esp32time.getDay(),
					.hour = (unsigned char)esp32time.getHour(),
					.minute = (unsigned char)esp32time.getMinute(),
					.second = (unsigned char)esp32time.getSecond()
				};
			}
		}
	#endif

	static SerialNumber serial_current;
	static off_t wait_position;

	namespace LORA {
		static void send_SEND(struct Payload_SEND const *const sending) {
			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_SEND));
			LoRa.write(uint8_t(DEVICE_ID));
			LORA::send_payload("SEND", sending, sizeof *sending);
			LoRa.endPacket(true);
		}
	}

	static class Resend : public Schedule {
	protected:
		unsigned int counter;
		struct Payload_SEND sending;
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
			LORA::send_SEND(&sending);
			if (!--counter) stop();
		}
		void start_send(struct Data const *const data) {
			sending.serial = serial_current;
			++serial_current;
			sending.data = *data;
			LORA::send_SEND(&sending);
			start(millis());
		}
		bool stop_ack(SerialNumber const serial) {
			if (serial == sending.serial) {
				stop();
				return true;
			} else {
				Serial_println("LoRa ACK: serial number unmatched");
				return false;
			}
		}
	} resend_schedule;

	namespace LORA {
		static void send(struct Data const *const data) {
			resend_schedule.start_send(data);
		}

		static bool send_ready(void) {
			return !resend_schedule.enabled();
		}

		static void receive_TIME(void) {
			struct DateTime payload;
			if (!LORA::receive_payload("TIME", &payload, sizeof payload))
				return;

			RTC::set(&payload);
		}
	}

	#ifdef ENABLE_SD_CARD
		static void writeln_Data /* FIX: stupid "feature" of Arduino IDE */ (class Print *const print, struct Data const *const data) {
			print->printf(
				"%04u-%02u-%02uT%02u:%02u:%02uZ",
				data->time.year, data->time.month, data->time.day,
				data->time.hour, data->time.minute, data->time.second
			);
			#ifdef ENABLE_DALLAS
				print->printf(",%f\n", data->dallas_temperature);
			#endif
			#ifdef ENABLE_BME
				print->printf(
					",%f,%f,%f",
					data->bme_temperature, data->bme_pressure, data->bme_humidity
				);
			#endif
			#ifdef ENABLE_LTR
				print->printf(",%f", data->ltr_ultraviolet);
			#endif
			print->write('\n');
		}

		static bool readln_Data /* FIX: stupid "feature" of Arduino IDE */ (struct Data *const data, class Stream *const stream) {
			/* Time */
			{
				class String const s = stream->readStringUntil(
					#if defined(ENABLE_DALLAS) || defined(ENABLE_BME) || defined(ENABLE_LTR)
						','
					#else
						'\n'
					#endif
				);
				if (
					sscanf(
						s.c_str(),
						"%4hu-%2hhu-%2hhuT%2hhu:%2hhu:%2hhuZ",
						&data->time.year, &data->time.month, &data->time.day,
						&data->time.hour, &data->time.minute, &data->time.second
					) != 6
				) return false;
			}
			/* Dallas Thermometer */
			#ifdef ENABLE_DALLAS
				{
					class String const s = stream->readStringUntil(
						#if defined(ENABLE_BME) || defined(ENABLE_LTR)
							','
						#else
							'\n'
						#endif
					);
					if (sscanf(s.c_str(), "%f", &data->dallas_temperature) != 1) return false;
				}
			#endif
			/* BME280 sensor */
			#ifdef ENABLE_BME
				{
					class String const s = stream->readStringUntil(',');
					if (sscanf(s.c_str(), "%f", &data->bme_temperature) != 1) return false;
				}
				{
					class String const s = stream->readStringUntil(',');
					if (sscanf(s.c_str(), "%f", &data->bme_pressure) != 1) return false;
				}
				{
					class String const s = stream->readStringUntil(
						#if defined(ENABLE_LTR)
							','
						#else
							'\n'
						#endif
					);
					if (sscanf(s.c_str(), "%f", &data->bme_humidity) != 1) return false;
				}
			#endif
			/* LTR390 sensor */
			#ifdef ENABLE_LTR
				{
					class String const s = stream->readStringUntil('\n');
					if (sscanf(s.c_str(), "%f", &data->ltr_ultraviolet) != 1) return false;
				}
			#endif
			return true;
		}

		static void dump_log_file(void) {
			Serial_println("Log file BEGIN");
			File file = SD.open(data_file_path, "r");
			for (;;) {
				signed int const c = file.read();
				if (c < 0) break;
				Serial_print(char(c));
			}
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
				class String const s = cleanup_file.readStringUntil(',');
				if (!s.length()) break;
				bool const sent = s != "0";

				struct Data data;
				if (!readln_Data(&data, &cleanup_file)) {
					Serial_println("Clean-up: invalid data");
					break;
				}

				if (!sent) {
					#ifdef DEBUG_CLEAN_OLD_DATA
						data_file.print("1,");
					#else
						data_file.print("0,");
					#endif
					writeln_Data(&data_file, &data);
				}
			}
			cleanup_file.close();
			data_file.close();
			SD.remove(cleanup_file_path);
		}

		static void append_data_file(struct Data const *const data) {
			File file = SD.open(data_file_path, "a", true);
			if (!file) {
				any_println("Cannot append data file");
			} else {
				file.print("0,");
				writeln_Data(&file, data);
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
				File data_file = SD.open(DATA_FILE_PATH, "r");
				if (!data_file) {
					Serial_println("Upload: fail to open data file");
					return;
				}
				if (!data_file.seek(position)) {
					Serial_print("Upload: cannot seek: ");
					Serial_println(position);
					data_file.close();
					return;
				}
				for (;;) {
					class String const s = data_file.readStringUntil(',');
					if (!s.length()) break;
					bool const sent = s != "0";
					struct Data data;
					if (!readln_Data(&data, &data_file)) {
						Serial_println("Upload: invalid data");
						break;
					}

					if (!sent) {
						wait_position = position;
						position = data_file.position();
						LORA::send(&data);
						break;
					}

					position = data_file.position();
				}
				data_file.close();
			}
		} upload_schedule;
	#endif

	static class Measure : public Schedule {
	public:
		inline Measure(void) : Schedule(MEASURE_INTERVAL) {}
		virtual void run(Time const now) {
			Schedule::run(now);
			if (!RTC::ready()) return;

			OLED_home();
			struct Data data;
			data.time = RTC::now();
			#ifdef ENABLE_COM_OUTPUT
				Serial.print("Time: ");
				Serial.printf(
					"%04u-%02u-%02uT%02u:%02u:%02uZ",
					data.time.year, data.time.month, data.time.day,
					data.time.hour, data.time.minute, data.time.second
				);
			#endif
			#ifdef ENABLE_DALLAS
				data.dallas_temperature = dallas.getTempCByIndex(0);
				any_print("Dallas temp.: ");
				any_println(data.dallas_temperature);
			#endif
			#ifdef ENABLE_BME
				data.bme_temperature = BME.readTemperature();
				data.bme_pressure = BME.readPressure();
				data.bme_humidity = BME.readHumidity();
				any_print("BME temp.: ");
				any_println(data.bme_temperature);
				any_print("BME pressure: ");
				any_println(data.bme_pressure, 0);
				any_print("BME humidity: ");
				any_println(data.bme_humidity);
			#endif
			#ifdef ENABLE_LTR
				data.ltr_ultraviolet = LTR.readUVS();
				any_print("LTR UV: ");
				any_println(data.ltr_ultraviolet);
			#endif

			#ifdef ENABLE_OLED_OUTPUT
				OLED_println(OLED_message);
				OLED_message = "";
			#endif
			#ifdef ENABLE_SD_CARD
				append_data_file(&data);
			#else
				LORA::send(&data);
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
		#ifdef ENABLE_SD_CARD
			upload_schedule.start(0);
		#endif
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

		/* Initialize Dallas thermometer */
		#ifdef ENABLE_DALLAS
			if (!setup_error) {
				dallas.begin();
				DeviceAddress thermometer_address;
				if (dallas.getAddress(thermometer_address, 0)) {
					any_println("Thermometer 0 found");
				} else {
					setup_error = true;
					any_println("Thermometer 0 not found");
				}
			}
		#endif

		/* Initialize BME280 sensor */
		#ifdef ENABLE_BME
			if (!setup_error) {
				if (BME.begin()) {
					any_println("BME280 sensor found");
				} else {
					setup_error = true;
					any_println("BME280 sensor not found");
				}
			}
		#endif

		/* Initial LTR390 sensor */
		#ifdef ENABLE_LTR
			if (!setup_error) {
				if (LTR.begin()) {
					LTR.setMode(LTR390_MODE_UVS);
					any_println("LTR390 sensor found");
				} else {
					setup_error = true;
					any_println("LTR390 sensor not found");
				}
			}
		#endif

		/* Initialize LoRa */
		if (!setup_error)
			setup_error = !LORA::initialize();

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
		RTC::initialize();

		/* display setup result on OLED */
		OLED_display();
	}

	namespace LORA {
		static void receive_ACK(void) {
			Device const device = LoRa.read();
			if (device != DEVICE_ID) return;
			SerialNumber serial;
			if (!receive_payload("ACK", &serial, sizeof serial)) return;
			if (!resend_schedule.stop_ack(serial)) return;

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

		static void receive(signed int const packet_size) {
			if (!packet_size) return;
			uint8_t const packet_type = LoRa.read();
			switch (packet_type) {
			case PACKET_SEND:
				/* SEND sent by other senders */
				break;
			case PACKET_TIME:
				if (packet_size != 1 + CIPHER_IV_LENGTH + sizeof (struct DateTime) + CIPHER_TAG_SIZE) {
					Serial_print("LoRa TIME: incorrect packet size: ");
					Serial_println(packet_size);
					break;
				}
				receive_TIME();
				break;
			case PACKET_ACK:
				if (packet_size != 1 + 1 + CIPHER_IV_LENGTH + sizeof (SerialNumber) + CIPHER_TAG_SIZE) {
					Serial_print("LoRa ACK: incorrect packet size: ");
					Serial_println(packet_size);
					break;
				}
				receive_ACK();
				break;
			default:
				Serial_print("LoRa: incorrect packet type: ");
				Serial_println(packet_type);
			}

			/* add entropy to RNG */
			unsigned long int const microseconds = micros();
			RNG.stir((uint8_t const *)&microseconds, sizeof microseconds, 8);
		}
	}

	void loop() {
		if (setup_error) {
			LED_flash();
			return;
		}
		LORA::receive(LoRa.parsePacket());
		measure_schedule.tick(millis());
		RNG.loop();
		LORA::receive(LoRa.parsePacket());
		resend_schedule.tick(millis());
		RNG.loop();
		#ifdef ENABLE_SD_CARD
			LORA::receive(LoRa.parsePacket());
			upload_schedule.tick(millis());
			RNG.loop();
		#endif
	}

#elif DEVICE_TYPE == DEVICE_RECEIVER /* ************************************* */

	#include <WiFi.h>
	#include <WiFiUdp.h>
	#include <HTTPClient.h>
	#include <NTPClient.h>

	static SerialNumber serial_last[NUMBER_OF_SENDERS];
	static signed int HTTP_status;
	static class WiFiUDP WiFiUDP;
	static class NTPClient NTP(WiFiUDP, NTP_SERVER);

	#ifndef ENABLE_CLOCK
		namespace RTC {
			inline static bool ready(void) {
				return NTP.isTimeSet();
			}

			static struct DateTime now(void) {
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
		}
	#endif

	static bool WiFi_upload(Device const device, SerialNumber const serial, struct Data const *const data) {
		signed int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return false;
		}
		class String const time = String_from_DateTime(&data->time);
		class HTTPClient HTTP_client;
		char URL[HTTP_UPLOAD_LENGTH];
		snprintf(
			URL, sizeof URL,
			HTTP_UPLOAD_FORMAT,
			device, serial, time.c_str()
			#ifdef ENABLE_DALLAS
				, data->dallas_temperature
			#endif
			#ifdef ENABLE_BME
				, data->bme_temperature
				, data->bme_pressure
				, data->bme_humidity
			#endif
			#ifdef ENABLE_LTR
				, data->ltr_ultraviolet
			#endif
		);
		Serial_print("Upload to ");
		Serial_println(URL);
		HTTP_client.begin(URL);
		static char const authorization_type[] = HTTP_AUTHORIZATION_TYPE;
		static char const authorization_code[] = HTTP_AUTHORIZATION_CODE;
		if (authorization_type[0] && authorization_code[0]) {
			HTTP_client.setAuthorizationType(authorization_type);
			HTTP_client.setAuthorization(authorization_code);
		}
		HTTP_status = HTTP_client.GET();
		Serial_print("HTTP status: ");
		Serial_println(HTTP_status);
		if (HTTP_status != 200) return false;
		return true;
	}

	static class String WiFi_status_message(signed int const WiFi_status) {
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

	namespace LORA {
		static void send_ACK(Device const device, SerialNumber const serial) {
			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_ACK));
			LoRa.write(uint8_t(device));
			LORA::send_payload("ACK", &serial, sizeof serial);
			LoRa.endPacket(true);
		}

		static void receive_SEND(void) {
			Device const device = LoRa.read();
			if (!(device > 0 && device <= NUMBER_OF_SENDERS)) return;
			struct Payload_SEND payload;
			if (!LORA::receive_payload("SEND", &payload, sizeof payload)) return;
			if (payload.serial < serial_last[device-1] && !(serial_last[device-1] & ~(~(SerialNumber)0 >> 1)))
				Serial_println("LoRa SEND: serial number out of order");
			class String const time = String_from_DateTime(&payload.data.time);
			serial_last[device-1] = payload.serial;
			#ifdef ENABLE_OLED_OUTPUT
				OLED_home();
			#endif
			if (!WiFi_upload(device, payload.serial, &payload.data)) {
				#ifdef ENABLE_OLED_OUTPUT
					OLED_println(WiFi_status_message(WiFi.status()));
					OLED_print("HTTP: ");
					OLED_println(HTTP_status);
				#endif
				OLED_println(OLED_message);
				OLED_message = "";
				OLED_display();
				return;
			}
			send_ACK(device, payload.serial);
			#ifdef ENABLE_OLED_OUTPUT
				signed int const WiFi_status = WiFi.status();
				OLED_print("Device ");
				OLED_print(device);
				OLED_print(" Serial ");
				OLED_println(payload.serial);
				OLED_println(time);
				#ifdef ENABLE_DALLAS
					OLED_print("Dallas temp.: ");
					OLED_println(payload.data.dallas_temperature);
				#endif
				#ifdef ENABLE_BME
					OLED_print("BME temp.: ");
					OLED_println(payload.data.bme_temperature);
					OLED_print("BME pressure: ");
					OLED_println(payload.data.bme_pressure, 0);
					OLED_print("BME humidity: ");
					OLED_println(payload.data.bme_humidity);
				#endif
				#ifdef ENABLE_LTR
					OLED_print("LTR UV: ");
					OLED_println(payload.data.ltr_ultraviolet);
				#endif
				OLED_println(OLED_message);
				OLED_message = "";
				OLED_display();
			#endif
		}

		static void receive(signed int const packet_size) {
			if (!packet_size) return;
			uint8_t const packet_type = LoRa.read();
			switch (packet_type) {
			case PACKET_SEND:
				if (packet_size != 1 + 1 + CIPHER_IV_LENGTH + sizeof (struct Payload_SEND) + CIPHER_TAG_SIZE) {
					Serial_print("LoRa SEND: incorrect packet size: ");
					Serial_println(packet_size);
					break;
				}
				receive_SEND();
				break;
			default:
				Serial_print("LoRa: incorrect packet type: ");
				Serial_println(packet_type);
			}

			/* add entropy to RNG */
			unsigned long int const microseconds = micros();
			RNG.stir((uint8_t const *)&microseconds, sizeof microseconds, 8);
		}
	}

	static class Synchronize : public Schedule {
	public:
		Synchronize(void) : Schedule(SYNCHONIZE_INTERVAL) {}
		virtual void run(Time const now) {
			Schedule::run(now);
			if (!RTC::ready()) return;
			struct DateTime const payload = RTC::now();

			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_TIME));
			LORA::send_payload("TIME", &payload, sizeof payload);
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
			setup_error = !LORA::initialize();
		}

		/* initialize WiFi */
		if (!setup_error) {
			WiFi.begin(WIFI_SSID, WIFI_PASS);
			//	any_println(WiFi_status_message(WiFi.status()));
		}

		/* initialize real-time clock */
		#ifdef ENABLE_CLOCK
			if (!setup_error) {
				RTC::external_clock.begin();
				RTC::external_clock.startClock();
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
		LORA::receive(LoRa.parsePacket());
		if (WiFi.status() == WL_CONNECTED) {
			if (NTP.update()) {
				#ifdef ENABLE_CLOCK
					time_t const epoch = NTP.getEpochTime();
					struct tm time;
					gmtime_r(&epoch, &time);
					RTC::external_clock.stopClock();
					RTC::external_clock.fillByYMD(1900+time.tm_year, time.tm_mon+1, time.tm_mday);
					RTC::external_clock.fillByHMS(time.tm_hour, time.tm_min, time.tm_sec);
					RTC::external_clock.setTime();
					RTC::external_clock.startClock();
				#endif
			}
		}
		synchronize_schedule.tick(millis());
		RNG.loop();
	}
#endif

/* ************************************************************************** */

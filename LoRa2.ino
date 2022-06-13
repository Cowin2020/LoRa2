/* *********************
LoRa sender and receiver
********************* */

#include <Arduino.h>

/* Network ID */
#define DEVICE_ID 0
#define NUMBER_OF_SENDERS 1

/* Feature Constants */
#define CLOCK_PCF85063TP 1
#define CLOCK_DS1307 2
#define CLOCK_DS3231 3
#define BATTERY_GAUGE_DFROBOT 1
#define BATTERY_GAUGE_LC709203F 2

/* Features */
#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
#define ENABLE_CLOCK CLOCK_DS3231
#define ENABLE_SD_CARD
#define ENABLE_BATTERY_GAUGE BATTERY_GAUGE_DFROBOT
#define ENABLE_DALLAS
#define ENABLE_BME
#define ENABLE_LTR

/* Software Parameters */
#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"
#define HTTP_UPLOAD_FORMAT \
	"http://www.example.com/upload?device=%1$u&serial=%2$u&time=%3$s" \
	"&voltage=%4$F&battery=%5$F" \
	"&dallas=%6$F" \
	"&temperature=%7$F&pressure=%8$F&humidity=%9$F" \
	"&ultraviolet=%10$F"
#define HTTP_UPLOAD_LENGTH 256
#define HTTP_AUTHORIZATION_TYPE ""
#define HTTP_AUTHORIZATION_CODE ""
#define NTP_SERVER "stdtime.gov.hk"
#define SECRET_KEY "This is secret!"
#define DATA_FILE_PATH "/data.csv"
#define CLEANUP_FILE_PATH "/cleanup.csv"
#define SYNCHONIZE_INTERVAL 7654321UL /* milliseconds */
#define RESEND_TIMES 3
#define ACK_TIMEOUT 1000UL /* milliseconds */
#define UPLOAD_INTERVAL 6000UL /* milliseconds */
#define MEASURE_INTERVAL 60000UL /* milliseconds */ /* MUST: > UPLOAD_INTERVAL */
#define ROUTER_TOPOLOGY {}

/* Hardware Parameters */
#define COM_BAUD 115200
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_I2C_ADDR 0x3C
#define DALLAS_PIN 3
#define LORA_PIN_RST 23  /* TTGO LoRa32 V2.1-1.6 version ? */
#define LORA_BAND 868000000

/* For Debug */
//	#define DEBUG_CLEAN_OLD_DATA

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
#include <cstring>
#include <memory>
#include <vector>

#include <RNG.h>
#include <AES.h>
#include <GCM.h>
//	#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#ifdef ENABLE_OLED_OUTPUT
	#include <Adafruit_SSD1306.h>
#endif

typedef uint8_t PacketType;
typedef unsigned long int Time;
typedef uint8_t Device;
typedef uint32_t SerialNumber;
typedef GCM<AES128> AuthCipher;

static Device const router_topology[][2] = ROUTER_TOPOLOGY;
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
		OLED.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
		OLED.clearDisplay();
		OLED.setCursor(0, 0);
	}

	inline static void OLED_home(void) {
		OLED.clearDisplay();
		OLED.setCursor(0, 0);
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
	inline void OLED_println(TYPE const x, int const option) {
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
	template <typename TYPE> inline void OLED_println(TYPE x, int option) {}
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

#ifdef ENABLE_COM_OUTPUT
void Serial_dump(char const *const label, void const *const memory, size_t const size) {
	Serial.printf("%s (%04X)", label, size);
	for (size_t i = 0; i < size; ++i) {
		unsigned char const c = i[(unsigned char const *)memory];
		Serial.printf(" %02X", c);
	}
	Serial.write('\n');
}
#else
inline static Serial_dump(void *const memory, size_t const size) {}
#endif

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

struct [[gnu::packed]] FullTime {
	unsigned short int year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
};

static class String String_from_FullTime(struct FullTime const *const fulltime) {
	char buffer[48];
	snprintf(
		buffer, sizeof buffer,
		"%04u-%02u-%02uT%02u:%02u:%02uZ",
		fulltime->year, fulltime->month, fulltime->day,
		fulltime->hour, fulltime->minute, fulltime->second
	);
	return String(buffer);
}

#ifdef ENABLE_CLOCK
	#if ENABLE_CLOCK == CLOCK_PCF85063TP
		#include <PCF85063TP.h>

		namespace RTC {
			class PCD85063TP external_clock;

			static bool initialize(void) {
				external_clock.begin();
				external_clock.startClock();
				return true;
			}

			static void set(struct FullTime const *const fulltime) {
				external_clock.stopClock();
				external_clock.fillByYMD(fulltime->year, fulltime->month, fulltime->day);
				external_clock.fillByHMS(fulltime->hour, fulltime->minute, fulltime->second);
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

			static struct FullTime now(void) {
				external_clock.getTime();
				return (struct FullTime){
					.year = (unsigned short int)(2000U + external_clock.year),
					.month = external_clock.month,
					.day = external_clock.dayOfMonth,
					.hour = external_clock.hour,
					.minute = external_clock.minute,
					.second = external_clock.second
				};
			}
		}
	#elif ENABLE_CLOCK == CLOCK_DS1307 || ENABLE_CLOCK == CLOCK_DS3231
		#include <RTClib.h>

		namespace RTC {
			#if ENABLE_CLOCK == CLOCK_DS1307
				class RTC_DS1307 external_clock;
			#elif ENABLE_CLOCK == CLOCK_DS3231
				class RTC_DS3231 external_clock;
			#endif

			static bool initialize(void) {
				if (!external_clock.begin()) {
					any_println("Clock not found");
					return false;
				}
				#if ENABLE_CLOCK == CLOCK_DS1307
					if (!external_clock.isrunning()) {
						any_println("DS1307 not running");
						return false;
					}
				#endif
				return true;
			}

			static void set(struct FullTime const *const fulltime) {
				class DateTime const datetime(
					fulltime->year, fulltime->month, fulltime->day,
					fulltime->hour, fulltime->minute, fulltime->second
				);
				external_clock.adjust(datetime);
			}

			static bool ready(void) {
				class DateTime const datetime = external_clock.now();
				return datetime.isValid();
			}

			static struct FullTime now(void) {
				class DateTime const datetime = external_clock.now();
				return (struct FullTime){
					.year = datetime.year(),
					.month = datetime.month(),
					.day = datetime.day(),
					.hour = datetime.hour(),
					.minute = datetime.minute(),
					.second = datetime.second()
				};
			}
		}
	#endif
#endif

class Schedule {
protected:
	bool enable;
	Time head;
	Time period;
	Time margin;
public:
	Schedule(Time initial_period);
	bool enabled(void) const;
	void start(Time now, Time addition_period = 0);
	void stop(void);
	virtual void run(Time now);
	bool tick(Time const now);
};

inline Schedule::Schedule(Time const initial_period) : enable(false), head(0), period(initial_period), margin(0) {}

inline bool Schedule::enabled(void) const {
	return enable;
}

inline void Schedule::start(Time const now, Time const addition_period) {
	enable = true;
	head = now;
	margin = addition_period;
}

inline void Schedule::stop(void) {
	enable = false;
}

inline void Schedule::run(Time const now) {
	head = now;
}

bool Schedule::tick(Time const now) {
	if (enable && now-head >= period+margin) {
		run(now);
		return true;
	} else {
		return false;
	}
}

class Schedules {
protected:
	class std::vector<class Schedule *> list;
public:
	Schedules(void);
	void add(class Schedule *schedule);
	void remove(class Schedule *schedule);
	void tick(void);
};

inline Schedules::Schedules(void) : list() {}

inline void Schedules::add(class Schedule *schedule) {
	list.push_back(schedule);
}

void Schedules::remove(class Schedule *schedule) {
	size_t const N = list.size();
	for (size_t i = 0; i < N; ++i) {
		class Schedule *p = list[i];
		if (p == schedule) {
			list[i] = list.back();
			list.pop_back();
			break;
		}
	}
}

void Schedules::tick(void) {
	Time const now = millis();
	for (class Schedule *schedule: list)
		if (schedule->tick(now))
			break;
}

static class Schedules schedules;

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
		uint8_t tag[CIPHER_TAG_SIZE];
		cipher.computeTag(tag, sizeof tag);
		LoRa.write(nonce, sizeof nonce);
		LoRa.write((uint8_t const *)&ciphertext, sizeof ciphertext);
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
		if (LoRa.readBytes(ciphertext, sizeof ciphertext) != sizeof ciphertext) {
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

struct [[gnu::packed]] Data {
	struct FullTime time;
	#if ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_DFROBOT || ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_LC709203F
		float battery_voltage;
		float battery_percentage;
	#endif
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
};

static bool setup_error;

/* ************************************************************************** */

#if DEVICE_TYPE == DEVICE_SENDER
	#ifdef ENABLE_SD_CARD
		#include <SD.h>

		static class SPIClass SPI_1(HSPI);
	#endif

	#if ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_DFROBOT
		#include <DFRobot_MAX17043.h>

		static class DFRobot_MAX17043 battery;
	#elif ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_LC709203F
		#include <Adafruit_LC709203F.h>

		static class Adafruit_LC709203F battery;
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
		#include <RTClib.h>

		namespace RTC {
			static bool clock_available;
			static class RTC_Millis internal_clock;

			static bool initialize(void) {
				clock_available = false;
				return true;
			}
			
			static void set(struct FullTime const *const fulltime) {
				class DateTime const datetime(
					fulltime->year, fulltime->month, fulltime->day,
					fulltime->hour, fulltime->minute, fulltime->second
				);
				if (clock_available) {
					internal_clock.adjust(datetime);
				} else {
					internal_clock.begin(datetime);
					clock_available = true;
				}
			}

			static bool ready(void) {
				return clock_available;
			}

			static struct FullTime now(void) {
				class DateTime const datetime = internal_clock.now();
				return (struct FullTime){
					.year = (unsigned short int)datetime.year(),
					.month = (unsigned char)datetime.month(),
					.day = (unsigned char)datetime.day(),
					.hour = (unsigned char)datetime.hour(),
					.minute = (unsigned char)datetime.minute(),
					.second = (unsigned char)datetime.second()
				};
			}
		}
	#endif

	static SerialNumber serial_current;
	static off_t wait_position;
	Device last_receiver;

	static class Resend : public Schedule {
	protected:
		unsigned int retry;
		Device receiver;
		SerialNumber serial;
		struct Data data;
		bool next_router(Device *next);
		void send_SEND(void);
	public:
		Resend(void);
		void start(Time const now);
		virtual void run(Time const now);
		void start_send(struct Data const *const data);
		bool stop_ack(SerialNumber const serial);
	} resend_schedule;

	Resend::Resend(void) : Schedule(ACK_TIMEOUT) {
		size_t const N = sizeof router_topology / sizeof *router_topology;
		size_t i = 0;
		for (size_t i = 0;; ++i) {
			if (i >= N) {
				last_receiver = 0;
				break;
			}
			if (router_topology[i][1] == Device(DEVICE_ID)) {
				last_receiver = router_topology[i][0];
				break;
			}
			++i;
		}
	}

	bool Resend::next_router(Device *const next) {
		size_t const N = sizeof router_topology / sizeof *router_topology;
		size_t i = 0;
		for (;;) {
			if (i >= N) return false;
			if (router_topology[i][1] == Device(DEVICE_ID) && router_topology[i][0] == receiver) break;
			++i;
		}
		size_t j = i + 1;
		for (;;) {
			if (j >= N) j = 0;
			if (router_topology[j][1] == Device(DEVICE_ID)) {
				Device const device = router_topology[j][0];
				if (device == last_receiver) return false;
				*next = device;
				return true;
			}
			++j;
		}
	}

	void Resend::send_SEND(void) {
		Device const device = DEVICE_ID;
		unsigned char payload[2 * sizeof device + sizeof serial + sizeof data];
		std::memcpy(payload, &device, sizeof device);
		std::memcpy(payload + sizeof device, &device, sizeof device);
		std::memcpy(payload + 2 * sizeof device, &serial, sizeof serial);
		std::memcpy(payload + 2 * sizeof device + sizeof serial, &data, sizeof data);
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_SEND));
		LoRa.write(uint8_t(receiver));
		LORA::send_payload("SEND", payload, sizeof payload);
		LoRa.endPacket(true);
	}

	void Resend::start(Time const now) {
		if (!RESEND_TIMES) return;
		uint8_t margin;
		RNG.rand(&margin, sizeof margin);
		Schedule::start(now, margin & 0xFF);
	}

	void Resend::run(Time const now) {
		Schedule::run(now);
		if (retry) {
			--retry;
			send_SEND();
		} else if (next_router(&receiver)) {
			retry = RESEND_TIMES;
		} else {
			stop();
		}
	}

	void Resend::start_send(struct Data const *const data) {
		retry = RESEND_TIMES;
		receiver = last_receiver;
		this->serial = serial_current;
		++serial_current;
		this->data = *data;
		Time const now = millis();
		start(now);
		run(now);
	}

	bool Resend::stop_ack(SerialNumber const serial) {
		if (serial == this->serial) {
			last_receiver = receiver;
			stop();
			return true;
		} else {
			Serial_println("LoRa ACK: serial number unmatched");
			return false;
		}
	}

	namespace LORA {
		static void send_data(struct Data const *const data) {
			resend_schedule.start_send(data);
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
			File file = SD.open(data_file_path, "r");
			if (!file) {
				Serial_print("Cannot open log file");
				return;
			}
			Serial_println("Log file BEGIN");
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
			File file = SD.open(data_file_path, "a");
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
						LORA::send_data(&data);
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
					"%04u-%02u-%02uT%02u:%02u:%02uZ\n",
					data.time.year, data.time.month, data.time.day,
					data.time.hour, data.time.minute, data.time.second
				);
			#endif
			#ifdef ENABLE_OLED_OUTPUT
				OLED.printf(
					"%04u-%02u-%02uT%02u:%02u:%02uZ\n",
					data.time.year, data.time.month, data.time.day,
					data.time.hour, data.time.minute, data.time.second
				);
			#endif
			#ifdef ENABLE_BATTERY_GAUGE
				#if ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_DFROBOT
					data.battery_voltage = battery.readVoltage() / 1000;
					data.battery_percentage = battery.readPercentage();
				#elif ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_LC709203F
					data.battery_voltage = battery.cellVoltage();
					data.battery_percentage = battery.cellPercent();
				#endif
				any_print("Battery ");
				any_print(data.battery_voltage);
				any_print("V ");
				any_print(data.battery_percentage);
				any_println("%");
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
				LORA::send_data(&data);
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
		schedules.add(&measure_schedule);
		schedules.add(&resend_schedule);
		#ifdef ENABLE_SD_CARD
			upload_schedule.start(0);
			schedules.add(&upload_schedule);
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

		/* Initial battery gauge */
		#if ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_DFROBOT || ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_LC709203F
			battery.begin();
		#endif

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
		if (!setup_error)
			setup_error = !RTC::initialize();

		/* display setup result on OLED */
		OLED_display();
	}

	namespace LORA {
		static void receive_TIME(signed int const packet_size) {
			signed int const excat_packet_size =
				sizeof (PacketType) +      /* packet type */
				sizeof (Device) +          /* sender */
				CIPHER_IV_LENGTH +         /* nonce */
				sizeof (struct FullTime) + /* time */
				CIPHER_TAG_SIZE;           /* cipher tag */
			if (packet_size != excat_packet_size) return;
			Device sender;
			if (LoRa.readBytes(&sender, sizeof sender) != sizeof sender) return;
			struct FullTime time;
			if (!LORA::receive_payload("TIME", &time, sizeof time)) return;

			if (sender != Device(0)) { /* always accept TIME packet from gateway */
				size_t i = 0;
				for (;;) {
					if (i >= sizeof router_topology / sizeof *router_topology) return;
					if (router_topology[i][0] == Device(DEVICE_ID) && router_topology[i][1] == sender) break;
					++i;
				}
			}

			RTC::set(&time);

			LoRa.beginPacket();
			LoRa.write(PacketType(PACKET_TIME));
			LoRa.write(Device(DEVICE_ID));
			LORA::send_payload("TIME+", &time, sizeof time);
			LoRa.endPacket(true);
		}

		static void receive_SEND(signed int const packet_size) {
			size_t const overhead_size =
				sizeof (PacketType) +   /* packet type */
				sizeof (Device) +       /* receiver */
				CIPHER_IV_LENGTH +      /* nonce */
				CIPHER_TAG_SIZE;        /* cipher tag */
			size_t const minimal_packet_size =
				sizeof (Device) +       /* terminal */
				sizeof (Device) +       /* router list length >= 1 */
				sizeof (SerialNumber) + /* serial code */
				sizeof (struct Data);   /* data */
			if (!(packet_size >= minimal_packet_size)) {
				Serial_print("LoRa SEND: incorrect packet size: ");
				Serial_println(packet_size);
				return;
			}
			Device receiver;
			if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;
			if (receiver != Device(DEVICE_ID)) return;
			unsigned char payload[sizeof (Device) + packet_size - overhead_size];
			if (!LORA::receive_payload("SEND", payload + 1, sizeof payload - 1)) return;

			std::memcpy(payload, payload + sizeof (Device), sizeof (Device));
			std::memcpy(payload + sizeof (Device), &receiver, sizeof (Device));
			LoRa.beginPacket();
			LoRa.write(PacketType(PACKET_SEND));
			LoRa.write(last_receiver);
			LORA::send_payload("SEND+", payload, sizeof payload);
			LoRa.endPacket(true);
		}

		static void receive_ACK(signed int const packet_size) {
			size_t const overhead_size =
				sizeof (PacketType) +   /* packet type */
				sizeof (Device) +       /* receiver */
				CIPHER_IV_LENGTH +      /* nonce */
				CIPHER_TAG_SIZE;        /* cipher tag */
			size_t const minimal_packet_size =
				overhead_size +
				sizeof (Device) +      /* terminal */
				sizeof (Device) +      /* router list length >= 1 */
				sizeof (SerialNumber); /* serial code */
			if (!(packet_size >= minimal_packet_size)) {
				Serial_print("LoRa ACK: incorrect packet size: ");
				Serial_println(packet_size);
				return;
			}
			Device receiver;
			if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;
			if (Device(DEVICE_ID) != receiver) return;
			unsigned char payload[packet_size - overhead_size];
			if (!LORA::receive_payload("ACK", payload, sizeof payload)) return;

			Device terminal, router0, router1;
			std::memcpy(&terminal, payload, sizeof terminal);
			std::memcpy(&router0, payload + sizeof terminal, sizeof router0);
			std::memcpy(&router1, payload + sizeof terminal + sizeof router0, sizeof router1);
			if (Device(DEVICE_ID) == terminal) {
				if (Device(DEVICE_ID) != router0) {
					Serial_print("LoRa ACK: dirty router list");
					return;
				}

				SerialNumber serial;
				std::memcpy(&serial, payload + 2 * sizeof (Device), sizeof serial);
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

				#ifdef ENABLE_OLED_OUTPUT
					//	OLED.setCursor(0, 54);
					//	OLED.print("ACK");
					OLED.drawRect(125, 61, 3, 3, SSD1306_WHITE);
					OLED.display();
				#endif
			} else {
				std::memcpy(payload + sizeof terminal, &terminal, sizeof terminal);
				LoRa.beginPacket();
				LoRa.write(PacketType(PACKET_ACK));
				LoRa.write(Device(router1));
				LORA::send_payload("ACK+", payload + sizeof terminal, sizeof payload - sizeof terminal);
				LoRa.endPacket(true);
			}
		}

		static void receive(signed int const packet_size) {
			if (packet_size < 1) return;
			Device packet_type;
			if (LoRa.readBytes(&packet_type, sizeof packet_type) != sizeof packet_type) return;
			switch (packet_type) {
			case PACKET_TIME:
				receive_TIME(packet_size);
				break;
			case PACKET_SEND:
				receive_SEND(packet_size);
				break;
			case PACKET_ACK:
				receive_ACK(packet_size);
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
		schedules.tick();
		RNG.loop();
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

			static struct FullTime now(void) {
				time_t const epoch = NTP.getEpochTime();
				struct tm time;
				gmtime_r(&epoch, &time);
				return (struct FullTime){
					.year = (unsigned short int)(1900U + time.tm_year),
					.month = (unsigned char)(time.tm_mon + 1),
					.day = (unsigned char)time.tm_mday,
					.hour = (unsigned char)time.tm_hour,
					.minute = (unsigned char)time.tm_min,
					.second = (unsigned char)time.tm_sec
				};
			}
			static void synchronize_NTP(void) {
				if (WiFi.status() == WL_CONNECTED) {
					if (NTP.update()) {
						#ifdef ENABLE_CLOCK
							time_t const epoch = NTP.getEpochTime();
							struct tm time;
							gmtime_r(&epoch, &time);
							struct FullTime const fulltime = {
								.year = (unsigned short int)(1900U + time.tm_year),
								.month = (unsigned char)(time.tm_mon + 1),
								.day = (unsigned char)time.tm_mday,
								.hour = (unsigned char)time.tm_hour,
								.minute = (unsigned char)time.tm_min,
								.second = (unsigned char)time.tm_sec
							};
							RTC::set(&fulltime);
						#endif
					}
				}
			}
		}
	#endif

	static bool WiFi_upload(Device const device, SerialNumber const serial, struct Data const *const data) {
		signed int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return false;
		}
		class String const time = String_from_FullTime(&data->time);
		class HTTPClient HTTP_client;
		char URL[HTTP_UPLOAD_LENGTH];
		snprintf(
			URL, sizeof URL,
			HTTP_UPLOAD_FORMAT,
			device, serial, time.c_str()
			#ifdef ENABLE_BATTERY_GAUGE
				, data->battery_voltage
				, data->battery_percentage
			#endif
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
		if (not (HTTP_status >= 200 and HTTP_status < 300)) return false;
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
		static void receive_SEND(signed int const packet_size) {
			size_t const overhead_size =
				sizeof (PacketType) +   /* packet type */
				sizeof (Device) +       /* receiver */
				CIPHER_IV_LENGTH +      /* nonce */
				CIPHER_TAG_SIZE;        /* cipher tag */
			size_t const minimal_packet_size =
				overhead_size +
				sizeof (Device) +       /* terminal */
				sizeof (Device) +       /* router list length >= 1 */
				sizeof (SerialNumber) + /* serial code */
				sizeof (struct Data);   /* data */
			if (!(packet_size >= minimal_packet_size)) {
				Serial_print("LoRa SEND: incorrect packet size: ");
				Serial_println(packet_size);
				return;
			}
			Device receiver;
			if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;
			if (receiver != Device(0)) return;
			size_t const payload_size = packet_size - overhead_size;
			unsigned char payload[payload_size];
			if (!LORA::receive_payload("SEND", &payload, sizeof payload)) return;

			Device device;
			std::memcpy(&device, payload, sizeof device);
			if (!(device > 0 && device <= NUMBER_OF_SENDERS)) {
				Serial_print("LoRa SEND: incorrect device: ");
				Serial_println(device);
				return;
			}

			size_t routers_length = sizeof (Device);
			for (;;) {
				if (routers_length >= payload_size) {
					Serial_println("LoRa SEND: incorrect router list");
					return;
				}
				Device router;
				std::memcpy(&router, payload + routers_length, sizeof router);
				if (router == device) break;
				routers_length += sizeof router;
			}
			size_t const excat_packet_size = minimal_packet_size + routers_length * sizeof (Device) - sizeof (Device);
			if (packet_size != excat_packet_size) {
				Serial_print("LoRa SEND: incorrect packet size or router list: ");
				Serial_print(packet_size);
				Serial_print(" / ");
				Serial_println(routers_length);
				return;
			}

			SerialNumber serial;
			std::memcpy(&serial, payload + sizeof (Device) + routers_length, sizeof serial);
			if (!(serial >= serial_last[device-1]) && !(serial_last[device-1] & ~(~(SerialNumber)0 >> 1)))
				Serial_println("LoRa SEND: serial number out of order");

			struct Data data;
			std::memcpy(&data, payload + sizeof (Device) * (1 + routers_length) + sizeof (SerialNumber), sizeof data);

			serial_last[device-1] = serial;
			#ifdef ENABLE_OLED_OUTPUT
				OLED_home();
			#endif

			if (!WiFi_upload(device, serial, &data)) {
				#ifdef ENABLE_OLED_OUTPUT
					OLED_println(WiFi_status_message(WiFi.status()));
					OLED_print("HTTP: ");
					OLED_println(HTTP_status);
				#endif
				OLED_println(OLED_message);
				OLED_message = "";
				OLED_display();
			}

			Device router;
			std::memcpy(&router, payload + sizeof device, sizeof router);

			LoRa.beginPacket();
			LoRa.write(PacketType(PACKET_ACK));
			LoRa.write(router);
			LORA::send_payload("ACK", payload, sizeof (Device) * (1 + routers_length) + sizeof (SerialNumber));
			LoRa.endPacket(true);

			#ifdef ENABLE_OLED_OUTPUT
				signed int const WiFi_status = WiFi.status();
				OLED_print("Device ");
				OLED_print(device);
				OLED_print(" Serial ");
				OLED_println(serial);
				OLED_println(String_from_FullTime(&data.time));
				#ifdef ENABLE_DALLAS
					OLED_print("Dallas temp.: ");
					OLED_println(data.dallas_temperature);
				#endif
				#ifdef ENABLE_BME
					OLED_print("BME temp.: ");
					OLED_println(data.bme_temperature);
					OLED_print("BME pressure: ");
					OLED_println(data.bme_pressure, 0);
					OLED_print("BME humidity: ");
					OLED_println(data.bme_humidity);
				#endif
				#ifdef ENABLE_LTR
					OLED_print("LTR UV: ");
					OLED_println(data.ltr_ultraviolet);
				#endif
				OLED_println(OLED_message);
				OLED_message = "";
				OLED_display();
			#endif
		}

		static void receive(signed int const packet_size) {
			if (!packet_size) return;
			Device packet_type;
			if (LoRa.readBytes(&packet_type, sizeof packet_type) != sizeof packet_type) return;
			switch (packet_type) {
			case PACKET_SEND:
				receive_SEND(packet_size);
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
			struct FullTime const payload = RTC::now();

			LoRa.beginPacket();
			LoRa.write(PacketType(PACKET_TIME));
			LoRa.write(Device(0));
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
		schedules.add(&synchronize_schedule);

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
		}

		/* initialize real-time clock */
		#ifdef ENABLE_CLOCK
			if (!setup_error) {
				setup_error = RTC::initialize();
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
		RTC::synchronize_NTP();
		schedules.tick();
		RNG.loop();
	}
#endif

/* ************************************************************************** */

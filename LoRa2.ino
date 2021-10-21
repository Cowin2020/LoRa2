/* *********************
LoRa sender and receiver
********************* */

#define DEVICE_SENDER 0  /* DEVICE_TYPE = 0 for sender */
#define DEVICE_RECEIVER 1  /* DEVICE_TYPE = 1 for receiver */
#define DEVICE_TYPE DEVICE_SENDER
#define DEVICE_ID 1
#define NUMBER_OF_SENDERS 1

#define WIFI_SSID "Mark1"
#define WIFI_PASS "5654345234345"
#define HTTP_URL "http://cowin.hku.hk:8765/"
static char const SECRET_KEY[16] = "This is secret!";
static char const AUTHENTICATION_DATA[] = "HKU CoWIN2 LoRa";
#define MEASURE_PERIOD 10000 /* milliseconds */
#define ACK_TIMEOUT 1000 /* milliseconds */
#define RESEND_LIMIT 5
#define IDLE_TIME 10000 /* milliseconds */

#define ENABLE_LED
#define ENABLE_COM_OUTPUT
#define ENABLE_OLED_OUTPUT
// #define X_ENABLE_LORA_CALLBACK /* not working? */

#define PIN_THERMOMETER 3
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define COM_BAUD 115200

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

#define PACKET_TIME   0
#define PACKET_ACK    1
#define PACKET_SEND   2
#define CIPHER_IV_LENGTH 16
#define CIPHER_TAG_SIZE 4

#include <SPI.h>
#include <LoRa.h>

#ifdef ENABLE_OLED_OUTPUT
	#include <Wire.h>
	#include <Adafruit_SSD1306.h>
#endif

#include <RNG.h>
#include <AES.h>
#include <GCM.h>

typedef unsigned long int Time;
typedef uint8_t Device;
typedef uint32_t SerialNumber;
typedef GCM<AES128> AuthCipher;

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

#ifdef ENABLE_OLED_OUTPUT
	static Adafruit_SSD1306 OLED(OLED_WIDTH, OLED_HEIGHT);
#endif

static bool LoRa_available;

#if DEVICE_TYPE == DEVICE_SENDER
	#include <OneWire.h>
	#include <DallasTemperature.h>
	#include <SD_MMC.h>

	static OneWire onewire_thermometer(PIN_THERMOMETER);
	static DallasTemperature thermometer(&onewire_thermometer);

	static SerialNumber serial_current;
	static bool SDCard_available;
	static float temperature;
	#ifdef ENABLE_OLED_OUTPUT
		static char const *OLED_message;
	#endif
	static SerialNumber serial_wait;

	static void send_LoRa_serial(SerialNumber const serial) {
		LoRa.beginPacket();
		LoRa.write(uint8_t(PACKET_SEND));
		LoRa.write(uint8_t(DEVICE_ID));
		uint8_t IV[16];
		RNG.rand(IV, sizeof IV);
		LoRa.write(IV, sizeof IV);
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
		if (!cipher.setIV(IV, sizeof IV)) {
			Serial_println("Unable to set IV");
			#ifdef ENABLE_OLED_OUTPUT
				OLED_message = "Unable to set IV";
			#endif
			return;
		}
		cipher.addAuthData(AUTHENTICATION_DATA, sizeof AUTHENTICATION_DATA);
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
			Schedule::start(now);
			counter = RESEND_LIMIT;
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
			OLED_display();
		}
	} measure_schedule;

	static void LoRa_receive_ACK(void) {
		Device const device = LoRa.read();
		if (device != DEVICE_ID) return;
		uint8_t IV[CIPHER_IV_LENGTH];
		if (LoRa.readBytes(IV, sizeof IV) != sizeof IV) {
			Serial_println("LoRa ACK: fail to read cipher IV");
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
		if (!cipher.setIV(IV, sizeof IV)) {
			Serial_println("LoRa ACK: fail to set cipher IV");
			#ifdef ENABLE_OLED_OUTPUT
				OLED_message = "LoRa ACK: fail to set cipher IV";
			#endif
			return;
		}
		cipher.addAuthData(AUTHENTICATION_DATA, sizeof AUTHENTICATION_DATA);
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

	static void LoRa_receive(int const packet_size) {
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
		serial_current = 0;
		OLED_message = "";
		measure_schedule.start(0);
		RNG.begin("LoRa-2");

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

		/* Initialize thermometer */
		//	pinMode(PIN_THERMOMETER, INPUT);
		thermometer.setWaitForConversion(true);
		thermometer.setCheckForConversion(true);

		/* Initialize LoRa */
		SPI.begin(PIN_LORA_SCLK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
		LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
		LoRa_available = LoRa.begin(LORA_BAND) == 1;
		if (LoRa_available) {
			any_println("LoRa initialized");
		} else {
			any_println("LoRa uninitialized");
		}

		/* initialize SD card */
		//	pinMode(PIN_SDCARD_MISO, INPUT_PULLUP);
		//	pinMode(4, INPUT_PULLUP);
		SDCard_available = SD_MMC.begin();
		if (SDCard_available) {
			any_println("SD card initialized");
			Serial_println(String("SD Card type: ") + String(SD_MMC.cardType()));
		} else {
			any_println("SD card uninitialized");
		}

		OLED_display();
	}

	void loop() {
		if (!LoRa_available || !SDCard_available) {
			#ifdef ENABLE_LED
				digitalWrite(PIN_LED, HIGH);
				delay(100);
				digitalWrite(PIN_LED, LOW);
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

	static int HTTP_status;
	#ifdef ENABLE_OLED_OUTPUT
		static uint8_t OLED_device;
		static SerialNumber OLED_serial;
		static float OLED_temperature;
	#endif

	void upload_WiFi(void) {
		int const WiFi_status = WiFi.status();
		if (WiFi_status != WL_CONNECTED) {
			Serial_println("Upload no WiFi");
			return;
		}
		HTTPClient HTTP_client;
		String const URL = String(HTTP_URL);
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
		uint8_t IV[16];
		RNG.rand(IV, sizeof IV);
		LoRa.write(IV, sizeof IV);
		SerialNumber cleantext;
		cleantext = serial;
		AuthCipher cipher;
		if (!cipher.setKey((uint8_t const *)SECRET_KEY, sizeof SECRET_KEY)) {
			Serial_println("LoRa SEND ACK: unable to set key");
			return;
		}
		if (!cipher.setIV(IV, sizeof IV)) {
			Serial_println("LoRa SEND ACK: unable to set IV");
			return;
		}
		cipher.addAuthData(AUTHENTICATION_DATA, sizeof AUTHENTICATION_DATA);
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
		uint8_t IV[CIPHER_IV_LENGTH];
		if (LoRa.readBytes(IV, sizeof IV) != sizeof IV) {
			Serial_println("LoRa SEND: fail to read cipher IV");
			return;
		}
		struct PayloadSend ciphertext;
		if (LoRa.readBytes((char *)&ciphertext, sizeof ciphertext) != sizeof ciphertext) {
			Serial_println("LoRa SEND: fail to read serial number");
			return;
		}
		AuthCipher cipher;
		if (!cipher.setKey((uint8_t const *)SECRET_KEY, sizeof SECRET_KEY)) {
			Serial_println("LoRa SEND: fail to set cipher key");
			return;
		}
		if (!cipher.setIV(IV, sizeof IV)) {
			Serial_println("LoRa SEND: fail to set cipher IV");
			return;
		}
		cipher.addAuthData(AUTHENTICATION_DATA, sizeof AUTHENTICATION_DATA);
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
		OLED_device = device;
		OLED_serial = cleantext.serial;
		OLED_temperature = cleantext.temperature;

		OLED_paint();
		upload_WiFi();
	}

	static void LoRa_receive(int const packet_size) {
		if (!packet_size) return;
		uint8_t const packet_type = LoRa.read();
		struct {
			SerialNumber serial;
			float temperature;
		} ciphertext, cleantext;
		uint8_t IV[CIPHER_TAG_SIZE];

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

		/* initialize WiFi */
		//WiFi.begin(WIFI_SSID, WIFI_PASS);
		OLED_println(WiFi_status_message(WiFi.status()));

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
		#ifndef ENABLE_LORA_CALLBACK
			LoRa_receive(LoRa.parsePacket());
		#endif
	}
#endif

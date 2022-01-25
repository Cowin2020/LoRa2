Install libraries
=================

Additional Board Manager URLs:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json

Crypto
------

Edit libraries/Crypto/src/AES.h
	remove the usage of "CRYPTO_AES_ESP32"
	always use "CRYPTO_AES_DEFAULT"

ESP32Time
---------

ESP8266 and ESP32 OLED driver for SSD1306 displays
--------------------------------------------------

NTPClient
---------

IDE version does not work.
It should be installed from github.
https://github.com/arduino-libraries/NTPClient.git

HTTPClient
----------

LoRa
----

OneWire
-------

DallasTemperature
-----------------

Grove - High Precision RTC
--------------------------

Configuration
=============

Create "config.h" in the root directory of repository.
Define or undefine variables for configuration.

DEVICE_ID
---------

ID of the arduino device

Each device should have a unique ID.

Type: Single byte integer
Values:
	0:     LoRa-WiFi gateway
	1-255: terminal measuring device

NUMBER_OF_SENDERS
-----------------

Number of terminal devices

Values: 1-255

ENABLE_LED
----------

Flash LED on error

Type: defined or undefined

ENABLE_COM_OUTPUT
-----------------

Output debug message to USB serial port

Type: defined or undefined

ENABLE_OLED_OUTPUT
------------------

Output message to OLED display

Type: defined or undefined

ENABLE_CLOCK
------------

Grove high precision RTC is used

Type: defined or undefined

ENABLE_SD_CARD
--------------

SD card is installed and used to store measurement data

Type: defined or undefined

ENABLE_DALLAS
-------------

Dallas thermometer is used

Type: defined or undefined

ENABLE_BME
----------

BME280 (temperature, pressure, humidity) sensor is used

Type: defined or undefined

ENABLE_LTR
----------

LTR390 I2C sensor is used

Type: defined or undefined

WIFI_SSID and WIFI_PASS
-----------------------

WiFi access point and password

Type: C strings

HTTP_UPLOAD_FORMAT
---------------

Printf format string of URL to upload data

Type: C string

Position arguments:
	%1$u
		device ID
	%2$lu
		serial number of transmission
	%3$s
		date and time in ISO8601 format
	%4$f, %5$f, %6$f, ...
		measure data, orderred by
			Dallas temperature
			BME280 temperature
			BME280 pressure
			BME280 humidity
			LTR390 ultraviolet
		data can be missing if corresponding ENABLE_* is not defined

HTTP_UPLOAD_LENGTH
---------------

Maximum length of HTTP URL string

SECRET_KEY
----------

Secret key for encryption of LoRa communication

Need to be same for both gateway and terminal sides.

Type: anything with 16 bytes size

DATA_FILE_PATH
-------------

File name in SD card to store data in terminal device

Type: C string

CLEANUP_FILE_PATH
-------------

Temporary file name in SD card for clean up data file

Type: C string

ACK_TIMEOUT
-----------

Milliseconds to wait for ACK response

Type: natural number

RESEND_TIMES
------------

Times to resend data if ACK is not received

Type: natural number

UPLOAD_INTERVAL
---------------

Maximum period in milliseconds to upload data if SD card is used
This value should be larger than (ACK_TIMEOUT * (RESEND_TIMES + 1)).

Type: natural number
Default: (ACK_TIMEOUT * (RESEND_TIMES + 2))

MEASURE_INTERVAL
--------------

Period in milliseconds to measure data

Type: natural number

DALLAS_PIN
----------

Pin number of thermometer data wire

Type: natural number

OLED_WIDTH and OLED_HEIGHT
--------------------------

The dimension of OLED display

Type: positive numbers

COM_BAUD
--------

Bit rate of USB serial port

Type: positive number

LORA_BAND
---------

Radio frequency used for LoRa

Type: signed long int

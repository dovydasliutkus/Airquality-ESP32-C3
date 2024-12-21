#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <NMEAGPS.h>
#include <sps30.h>
#include <ThingSpeak.h>
#include <WiFi.h>

#define DEBUG_ENABLED
#define SerialMonitor Serial
#define SerialGPS Serial1

#ifdef DEBUG_ENABLED
#define DEBUG_PRINTLN(x) SerialMonitor.println(x)
#define DEBUG_PRINT(x) SerialMonitor.print(x)
#define DEBUG_PRINTF(format, ...) SerialMonitor.printf(format, __VA_ARGS__)  // Define macro with variadic arguments
#else
#define DEBUG_PRINTLN(x)           // Empty define (no operation)
#define DEBUG_PRINT(x)             // Empty define (no operation)
#define DEBUG_PRINTF(format, ...)  // Empty define (no operation)
#endif

#define RX1 10
#define TX1 4
#define SDA_PIN 5
#define SCL_PIN 6

// For BME280
#define SEALEVELPRESSURE_HPA (1013.25)

#define CHANNEL_NUMBER 2
#define CHANNEL_API_KEY "IR8FECNZANR3UUJ0"

const char* ssid = "Dovydoo";   
const char* password = "0987654321";   // your network password

WiFiClient  client;

void setupSPS();
void setupBME();
// TODO: add readSPS();
bool readGPS();
void readBatteryVoltage();
void readBME();
void printSPSData();
void printGPSInfo();

NMEAGPS GPS;
gps_fix GPS_fix;
Adafruit_BME280 BME;  // I2C

struct {
  // GPS readings
  int timestamp = 0;
  float latitude = 0.0;
  float longitude = 0.0;
  // Air particulate matter sensor readings
  sps30_measurement airData;
  float battery_voltage;
  // BME280 readings
  float temperature;  // In deg celcius
  float humidity;     // in %
  float pressure;     // in hPa
} AllData;

hw_timer_t * timer = NULL; // Timer interrupt handler
volatile bool timerFlag = false; // Volatile because it can change outside normal program execution (ISR)

void IRAM_ATTR onTimer() { // Place ISR in IRAM for fast access
  timerFlag = true;
}

void setup() {
#ifdef DEBUG_ENABLED
  SerialMonitor.begin(115200);
#endif
  SerialGPS.begin(9600,SERIAL_8N1,RX1,TX1); // GPS UART

  Wire.begin(SDA_PIN, SCL_PIN); //I2C
  setupSPS();
  setupBME();

  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client); 
  DEBUG_PRINTLN("Setup Complete");

  // Initialize the timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, onTimer, true); // Attach interrupt to timer
  timerAlarmWrite(timer, 1000000, true); // Set alarm to trigger every 1 second (1 million microseconds)
  timerAlarmEnable(timer); // Enable the timer alarm
}

void loop() {
  uint16_t ret;
  uint16_t data_ready = 0;
  if(timerFlag){
    timerFlag = 0;
    readGPS();

    while (!data_ready or ret < 0) {             // Wait for sps30 new data
      ret = sps30_read_data_ready(&data_ready);  // Indicates that new (not yet retrieved) data is ready
      if (ret < 0) {
        DEBUG_PRINT("error reading data-ready flag: ");
        DEBUG_PRINTLN(ret);
        ESP.restart();
      } else if (!data_ready) {
        DEBUG_PRINTLN("data not ready, no new measurement available");
      }
      delay(100);
   };

    ret = sps30_read_measurement(&AllData.airData);
    if (ret < 0) {
      DEBUG_PRINTLN("error reading measurement");
      ESP.restart();
    } 
    else {
      printSPSData();
      readBatteryVoltage();
      readBME();
      //Connect or reconnect to WiFi
      if(WiFi.status() != WL_CONNECTED){
        DEBUG_PRINTLN("Attempting to connect");
        while(WiFi.status() != WL_CONNECTED){
          WiFi.begin(ssid, password); 
          delay(100);     
        } 
        DEBUG_PRINTLN("\nConnected.");
      }
      ThingSpeak.setField(1, AllData.airData.mc_1p0);
      ThingSpeak.setField(2, AllData.airData.mc_2p5);
      ThingSpeak.setField(3, AllData.airData.mc_10p0);
      ThingSpeak.setField(4, AllData.temperature);
      ThingSpeak.setField(5, AllData.humidity);
      ThingSpeak.setField(6, AllData.battery_voltage);

      int responseCode = ThingSpeak.writeFields(CHANNEL_NUMBER, CHANNEL_API_KEY);

      if (responseCode == 200) {
          DEBUG_PRINTLN("Fields updated successfuly.");
      } else {
          DEBUG_PRINTLN("Problem updating fields. HTTP error code " + String(responseCode));
      }
    }  
  }
}

void setupSPS() {
  uint8_t auto_clean_days = 1;

  sensirion_i2c_init();  // Initialize all hard- and software components that are needed for the I2C comms

  // ALL SPS functions return 0 on success
  while (sps30_probe() != 0) {  // Check if SPS sensor is connected and initialize
    DEBUG_PRINTLN("SPS sensor probing failed");
    delay(500);
  }

  int16_t ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
  if (ret) {
    DEBUG_PRINT("error setting the auto-clean interval: ");
    DEBUG_PRINTLN(ret);
  }

  ret = sps30_start_measurement();
  if (ret < 0) {
    DEBUG_PRINTLN("error starting measurement");
  }
  // delay(1000); // SPS measurement takes 1s
}

void setupBME() {
  bool status = BME.begin(0x76);

  if (!status) {
    DEBUG_PRINTLN("Could not find a valid BME280 sensor, check wiring!");
  }
}

bool readGPS() {
  while (GPS.available(SerialGPS)) {
    GPS_fix = GPS.read();  // Read one byte and check if a GPS sentence is formed

    printGPSInfo();

    if (GPS_fix.valid.location && GPS_fix.valid.date && GPS_fix.valid.time) {
      AllData.latitude = GPS_fix.latitude();
      AllData.longitude = GPS_fix.longitude();
      return true;
    } else {
      if (millis() > 5000 && GPS.statistics.chars < 10) {
        DEBUG_PRINTLN(F("No GPS detected: check wiring."));
      }
      return false;
    }
  }
  return false;
}

void readBatteryVoltage() {
  int ADCValue;
  float ADCVoltage;
  ADCValue = analogRead(1);                                 // Read adcvalue
  ADCVoltage = ADCValue * 2.85 / 4095;                        // Convert from 0-4095 to 0-3.3 (to voltage) ESP32-C3 has 12bit ADCs
  AllData.battery_voltage = ADCVoltage * (220 + 750) / 220;  // Compensate for the voltage divider

  DEBUG_PRINTF("ADC Value: %d\n\r", ADCValue);
  DEBUG_PRINTF("Battery voltage: %f\n\rV", AllData.battery_voltage);
}

void readBME() {
  AllData.temperature = BME.readTemperature();
  AllData.humidity = BME.readHumidity();
  AllData.pressure = BME.readPressure() / 100.0F;

  DEBUG_PRINTF("Temperature = %f C ", AllData.temperature);
  DEBUG_PRINTF("Humidity: %f%% ", AllData.humidity);
  DEBUG_PRINTF("Pressure: %f \n", AllData.pressure);
}

void printSPSData() {
  DEBUG_PRINT("PM  1.0: ");
  DEBUG_PRINTLN(AllData.airData.mc_1p0);
  DEBUG_PRINT("PM  2.5: ");
  DEBUG_PRINTLN(AllData.airData.mc_2p5);
  DEBUG_PRINT("PM  4.0: ");
  DEBUG_PRINTLN(AllData.airData.mc_4p0);
  DEBUG_PRINT("PM 10.0: ");
  DEBUG_PRINTLN(AllData.airData.mc_10p0);

  DEBUG_PRINT("NC  0.5: ");
  DEBUG_PRINTLN(AllData.airData.nc_0p5);
  DEBUG_PRINT("NC  1.0: ");
  DEBUG_PRINTLN(AllData.airData.nc_1p0);
  DEBUG_PRINT("NC  2.5: ");
  DEBUG_PRINTLN(AllData.airData.nc_2p5);
  DEBUG_PRINT("NC  4.0: ");
  DEBUG_PRINTLN(AllData.airData.nc_4p0);
  DEBUG_PRINT("NC 10.0: ");
  DEBUG_PRINTLN(AllData.airData.nc_10p0);

  DEBUG_PRINT("Typical particle size: ");
  DEBUG_PRINTLN(AllData.airData.typical_particle_size);
  DEBUG_PRINTLN();
}

void printGPSInfo() {
  DEBUG_PRINT(F("Location: "));
  if (GPS_fix.valid.location) {
    DEBUG_PRINTF("Lat: %lf,Lng: %lf, Time:%d:%d:%d, Date value: %d\n\r", GPS_fix.latitude(),
                 GPS_fix.longitude(), GPS_fix.dateTime.hours, GPS_fix.dateTime.minutes, GPS_fix.dateTime.seconds, GPS_fix.dateTime.date);
  } else {
    DEBUG_PRINTLN(F("INVALID"));
  }
}

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
///#include <NMEAGPS.h>
#include <SparkFun_u-blox_GNSS_v3.h>
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
void setupGPS();
bool readGPS();
void connectWifi();
void readBatteryVoltage();
void readBME();
void printSPSData();
void printGPSInfo();

SFE_UBLOX_GNSS_SERIAL myGNSS;
// NMEAGPS GPS;
// gps_fix GPS_fix;
Adafruit_BME280 BME;  // I2C

struct {
  // GPS readings
  //int timestamp = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  uint8_t SIV;
  // Air particulate matter sensor readings
  sps30_measurement airData;
  float battery_voltage;
  // BME280 readings
  float temperature;  // In deg celcius
  float humidity;     // in %
  float pressure;     // in hPa
} AllData;

hw_timer_t * timer = NULL; // Timer interrupt handler
volatile bool timerFlag = 0; // Volatile because it can change outside normal program execution (ISR)

void IRAM_ATTR onTimer() { // Place ISR in IRAM for fast access
  timerFlag = 1;
}

void setup() {
#ifdef DEBUG_ENABLED
  SerialMonitor.begin(115200);
#endif
  // SerialGPS.begin(38400,SERIAL_8N1,RX1,TX1); // GPS UART
  setupGPS();
  
  Wire.begin(SDA_PIN, SCL_PIN); //I2C
  setupSPS();
  setupBME();

  WiFi.useStaticBuffers(true);
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client); 
  connectWifi();

  // Initialize the timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, onTimer, true); // Attach interrupt to timer
  timerAlarmWrite(timer, 2000000, true); // Set alarm to trigger every 1 second (1 million microseconds)
  timerAlarmEnable(timer); // Enable the timer alarm

  DEBUG_PRINTLN("Setup Complete");
}

void loop() {
  uint16_t ret;
  uint16_t data_ready = 0;
  if(timerFlag){
    DEBUG_PRINTF("LOOP START: %ld ms\n",millis());
    timerFlag = 0;
    //SPS30 Read
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
      while (!readGPS()){
        DEBUG_PRINTLN("Reading GPS");
        delay(100);
      }
      printSPSData();
      readBatteryVoltage();
      readBME();
      DEBUG_PRINTF("FINISHED ALL READS: %ld ms\n",millis());
      connectWifi();
      ThingSpeak.setField(1, AllData.airData.mc_1p0);
      ThingSpeak.setField(2, AllData.airData.mc_2p5);
      ThingSpeak.setField(3, AllData.airData.mc_10p0);
      ThingSpeak.setField(4, AllData.temperature);
      ThingSpeak.setField(5, AllData.humidity);
      ThingSpeak.setField(6, AllData.battery_voltage);

      int responseCode = ThingSpeak.writeFields(CHANNEL_NUMBER, CHANNEL_API_KEY);

      if (responseCode == 200) {
          DEBUG_PRINTLN("Fields updated successfuly.\n\n");
      } else {
          DEBUG_PRINTLN("Problem updating fields. HTTP error code" + String(responseCode) + "\n\n");
      }
      DEBUG_PRINTF("FINISHED UPLOAD: %ld ms\n",millis());
    }  
  }
}

void connectWifi() {
 //Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    DEBUG_PRINTLN("Attempting to connect");
    WiFi.begin(ssid, password); 
    while(WiFi.status() != WL_CONNECTED){
      delay(100);     
      Serial.print(".");
    } 
    DEBUG_PRINTLN("\nConnected.");
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
}

void setupBME() {
  bool status = BME.begin(0x76);

  if (!status) {
    DEBUG_PRINTLN("Could not find a valid BME280 sensor, check wiring!");
  }
}
void setupGPS() {
  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    Serial.println("GNSS: trying 38400 baud");
    SerialGPS.begin(38400,SERIAL_8N1,RX1,TX1);
    if (myGNSS.begin(SerialGPS) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    SerialGPS.begin(9600,SERIAL_8N1,RX1,TX1);
    if (myGNSS.begin(SerialGPS) == true) {
        //myGNSS.factoryDefault();
        Serial.println("GNSS: connected at 9600 baud, switching to 38400");
        myGNSS.setSerialRate(38400);
        delay(100);
    } else {
        //myGNSS.factoryDefault();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");
  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
   // Set measurement to 10 Hz (T=100ms)
  myGNSS.setMeasurementRate(100);
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
}

// bool readGPS() {
// // GPS read for NEOGPS library
//   while (GPS.available(SerialGPS)) {
//     GPS_fix = GPS.read();  // Read one byte and check if a GPS sentence is formed

//     printGPSInfo();

//     if (GPS_fix.valid.location && GPS_fix.valid.date && GPS_fix.valid.time) {
//       AllData.latitude = GPS_fix.latitude();
//       AllData.longitude = GPS_fix.longitude();
//       return true;
//     } else {
//       if (millis() > 5000 && GPS.statistics.chars < 10) {
//         DEBUG_PRINTLN(F("No GPS detected: check wiring."));
//       }
//       return false;
//     }
//   }
//   return false;
// }
bool readGPS(){
  // GPS read with sparkfun library
   if (myGNSS.getPVT() == true) // Returns true if new data is available
  {
    int32_t latitude = myGNSS.getLatitude();
    AllData.latitude = latitude / 10000000.0;
    int32_t longitude = myGNSS.getLongitude();
    AllData.longitude = longitude / 10000000.0;
    AllData.SIV = myGNSS.getSIV();

    DEBUG_PRINTF("Lat: %9.7f Long: %9.7f SIV: %d\n",AllData.latitude,AllData.longitude,AllData.SIV);

    return 1;
  }
  else
    return 0;
}
void readBatteryVoltage() {
  int ADCValue;
  float ADCVoltage;
  ADCValue = analogRead(1);                                 // Read adcvalue
  ADCVoltage = ADCValue * 3.0 / 4095;                        // Convert from 0-4095 to 0-3.3 (to voltage) ESP32-C3 has 12bit ADCs
  AllData.battery_voltage = ADCVoltage * (10 + 43.1) / 10;  // Compensate for the voltage divider

  DEBUG_PRINTF("ADC Value: %d\n\r", ADCValue);
  DEBUG_PRINTF("Battery voltage: %3.1f V\n\r", AllData.battery_voltage);
}

void readBME() {
  AllData.temperature = BME.readTemperature();
  AllData.humidity = BME.readHumidity();
  AllData.pressure = BME.readPressure() / 100.0F;

  DEBUG_PRINTF("Temperature = %f C ", AllData.temperature);
  DEBUG_PRINTF("Humidity: %f%% ", AllData.humidity);
  DEBUG_PRINTF("Pressure: %f hPa\n", AllData.pressure);
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
}

// void printGPSInfo() {
//   DEBUG_PRINT(F("Location: "));
//   if (GPS_fix.valid.location) {
//     DEBUG_PRINTF("Lat: %lf,Lng: %lf, Time:%d:%d:%d, Date value: %d\n\r", GPS_fix.latitude(),
//                  GPS_fix.longitude(), GPS_fix.dateTime.hours, GPS_fix.dateTime.minutes, GPS_fix.dateTime.seconds, GPS_fix.dateTime.date);
//   } else {
//     DEBUG_PRINTLN(F("INVALID"));
//   }
// }

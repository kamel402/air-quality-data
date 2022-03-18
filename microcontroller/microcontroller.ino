// Interfacing Arduino with NEO-6M GPS module

#include <TinyGPS++.h>           // Include TinyGPS++ library
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <WebServer.h>
#include <MPU6050_tockn.h>
#include <Arduino_JSON.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Sds011.h>

#include <GxEPD.h>
// Genutztes Display 200x200 1.54inch Schwarz / Weiss E-Ink Display
// https://www.bastelgarage.ch/index.php?route=product/product&path=67_84&product_id=428
#include <GxGDEP015OC1/GxGDEP015OC1.h>    // 1.54" b/w
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include "IMG_0001.h"

// Genutzte Schrift importieren
// https://learn.adafruit.com/adafruit-gfx-graphics-library/using-fonts

#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMono12pt7b.h>

#define TIMEINTERVALL 1  60* 1000

TinyGPSPlus gps;
Adafruit_BME280 bme; // I2C
#define S_RX    2                // Define software serial RX pin for GPS
#define S_TX    19              // Define software serial TX pin
#define RXD2 26                 //SDS011
#define TXD2 27
#define SEALEVELPRESSURE_HPA (1013.25)
#define SDA 21
#define SCL 22
#define period  5000
#define DATAARRAYSIZE 100

TwoWire I2C = TwoWire(0);

String node = "NODE-IP";

MPU6050 mpu6050(I2C);
HardwareSerial GPSSerial(2);    // Configure GPSSerial library
HardwareSerial serialSDS(1);
Sds011Async< HardwareSerial > sds011(serialSDS);
GxIO_Class io(SPI, SS, 17, 16);  //SPI,SS,DC,RST
GxEPD_Class display(io, 16, 4);  //io,RST,BUSY
unsigned status;
const char* ssid = "SSID";
const char* password = "PASSWORD";
struct DATA {
  double Pm25;
  double Pm10;
  double temperature;
  double humidity;
  double pressure;
  double GPS_latitude;
  double GPS_longitude;
  double GPS_altitude;
  double GPS_speed;
  double accX;
  double accY;
  double accZ;
  String timestamp;
};

unsigned int dataArrayIndex = 0;
unsigned int dataArraySize = 0;
unsigned long time_now = 0;
DATA dataArray[DATAARRAYSIZE];

boolean GPS_available = false;
bool is_SDS_running = true;
boolean connectedToWifi = false;

HTTPClient http;
double maxAccX = 0;
double maxAccY = 0;
double maxAccZ = 0;


void BME280Values(DATA *d) {
  if (status) {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    d->temperature = bme.readTemperature();
    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    d->pressure = bme.readPressure() / 100.0F;
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    d->humidity = bme.readHumidity();
    Serial.println();
  }
}
void MPU6050Values(DATA *d) {
  mpu6050.update();
  Serial.print("temp : "); Serial.println(mpu6050.getTemp());
  Serial.print("accX : "); Serial.print(mpu6050.getAccX());
  Serial.print("\taccY : "); Serial.print(mpu6050.getAccY());
  Serial.print("\taccZ : "); Serial.println(mpu6050.getAccZ());
  d->accZ = mpu6050.getAccZ();
  d->accY  = mpu6050.getAccY();
  d->accX =  mpu6050.getAccX();

  Serial.print("gyroX : "); Serial.print(mpu6050.getGyroX());
  Serial.print("\tgyroY : "); Serial.print(mpu6050.getGyroY());
  Serial.print("\tgyroZ : "); Serial.println(mpu6050.getGyroZ());

  Serial.print("accAngleX : "); Serial.print(mpu6050.getAccAngleX());
  Serial.print("\taccAngleY : "); Serial.println(mpu6050.getAccAngleY());

  Serial.print("gyroAngleX : "); Serial.print(mpu6050.getGyroAngleX());
  Serial.print("\tgyroAngleY : "); Serial.print(mpu6050.getGyroAngleY());
  Serial.print("\tgyroAngleZ : "); Serial.println(mpu6050.getGyroAngleZ());

  Serial.print("angleX : "); Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : "); Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : "); Serial.println(mpu6050.getAngleZ());
}
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (GPSSerial.available())
      gps.encode(GPSSerial.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}
void service() {
  if (millis() > time_now + period) {
    time_now = millis();
    connectedToWifi = connectWifi();
    DATA d;
    processSDS011Data(&d);
    GPSData(&d);
    BME280Values(&d);
    MPU6050Values(&d);
    if (gps.location.isValid()) {
      dataInArray(&d);
      Serial.println("Array appended");
    }
    updateDisplay(d);
    if ( connectedToWifi ) { //Check the current connection status
      sendData();
    }
    Serial.println("End Loop");
  }
}
boolean GPSData(DATA *d) {
  if (gps.location.isValid()) {
    Serial.print("Latitude   = ");
    Serial.println(gps.location.lat(), 6);
    d->GPS_latitude = gps.location.lat();
    Serial.print("Longitude  = ");
    Serial.println(gps.location.lng(), 6);
    d->GPS_longitude = gps.location.lng();
    GPS_available = true;
  }
  else {
    Serial.println("Location Invalid");
    GPS_available = false;
  }

  if (gps.altitude.isValid()) {
    Serial.print("Altitude   = ");
    Serial.print(gps.altitude.meters());
    Serial.println(" meters");
    d->GPS_altitude = gps.altitude.meters();
  }
  else {
    Serial.println("Altitude Invalid");
    GPS_available = false;
  }

  if (gps.speed.isValid()) {
    Serial.print("Speed      = ");
    Serial.print(gps.speed.kmph());
    Serial.println(" kmph");
    d->GPS_speed = gps.speed.kmph();
  }
  else
    Serial.println("Speed Invalid");

  if (gps.time.isValid()) {
    Serial.print("Time (GMT) : ");
    int h = (gps.time.hour() + 2) % 24;
    if (h < 10)     Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10)   Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10)   Serial.print("0");
    Serial.println(gps.time.second());

  }
  else
    Serial.println("Time Invalid");

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print("Date       : ");
    if (gps.date.day() < 10)      Serial.print("0");
    Serial.print(gps.date.day());
    Serial.print("/");
    if (gps.date.month() < 10)    Serial.print("0");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());
    String dm = "";
    if (gps.date.month() < 10) dm = "0";
    String ddd = "";
    if (gps.date.day() < 10) ddd = "0";
    String dh = "";
    if (gps.time.hour() < 10) dh = "0";
    String dmm = "";
    if (gps.time.minute() < 10) dmm = "0";
    String ds = "";
    if (gps.time.second() < 10) ds = "0";
    String y =  String(gps.date.year() );
    String m  =   dm + String(gps.date.month());
    String dd = ddd + String(gps.date.day());
    String h =  dh + String(gps.time.hour());
    String mm =  dmm + String( gps.time.minute());
    String s =  ds + String(gps.time.second());
    Serial.println(y);
    Serial.println(m);
    Serial.println(dd);
    Serial.println(h);
    Serial.println(mm);
    Serial.println(s);
    //long date1 = y * 1000000000000+ m* 100000000+ dd* 1000000+h*10000+mm * 100+s;
    // Serial.println(date1);
    d->timestamp = "" + y +  m + dd + h + mm + s;

  }
  if (gps.satellites.isValid()) {
    Serial.print("Satellites = ");
    Serial.println(gps.satellites.value());
    if (gps.satellites.value())
      return true;
  }
  else {
    Serial.println("Satellites Invalid");
    return false;
  }
  return true;
}
void processSDS011Data(DATA *d) {
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;
  while (serialSDS.available() > 0)
  {
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
    mData = serialSDS.read();     delay(2);//wait until packet is received
    //Serial.println(mData);
    //Serial.println("*");
    if (mData == 0xAA) //head1 ok
    {
      mPkt[0] =  mData;
      mData = serialSDS.read();
      if (mData == 0xc0) //head2 ok
      {
        mPkt[1] =  mData;
        mCheck = 0;
        for (i = 0; i < 6; i++) //data recv and crc calc
        {
          mPkt[i + 2] = serialSDS.read();
          delay(2);
          mCheck += mPkt[i + 2];
        }
        mPkt[8] = serialSDS.read();
        delay(1);
        mPkt[9] = serialSDS.read();
        if (mCheck == mPkt[8]) //crc ok
        {
          serialSDS.flush();
          //Serial.write(mPkt,10);

          d->Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3] << 8);
          d->Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5] << 8);
          if (d->Pm25 > 9999)
            d->Pm25 = 9999;
          if (d->Pm10 > 9999)
            d->Pm10 = 9999;
          Serial.print("Pm25:");
          Serial.println(d->Pm25);
          Serial.print("Pm10:");
          Serial.println(d->Pm10);
          return;
        }
      }
    }
  }
}

void dataInArray(DATA *d) {
  if (dataArrayIndex == DATAARRAYSIZE ) {
    dataArrayIndex = 0;
  }
  dataArray[ dataArrayIndex] = *d;
  Serial.println("Append Array");
  if (dataArraySize != DATAARRAYSIZE) {
    dataArraySize++;
    dataArrayIndex++;
    Serial.print("Size: ");
    Serial.println(dataArraySize);
  }
  else {
    dataArrayIndex = 0;
  }
}
void sendData() {
  if (dataArraySize != 0) {
    http.begin(node);
    http.addHeader("Content-Type", "application/json");

    String req = "[";
    req = String("{\"pwd\":\"PASSWORD\",\"data\":[");
    for (int i = 0; i < dataArraySize; i++) {
      //Serial.println("Data to Json");
      //Serial.println(i);
      //Serial.println(dataArray[i].humidity);
      if (i != 0)
        req += ",";
      req += toJSON(dataArray[i]);
      Serial.println("...");
    }

    req += "]}";
    Serial.println("Send POST...");
    Serial.print("Size: ");
    Serial.println(dataArraySize);
    int httpCode = http.POST(req);
    Serial.println(req);
    if (httpCode == 200 || httpCode == -11) { //Check for the returning code
      dataArrayIndex = 0; //Clear Array
      dataArraySize = 0;
      String output;
      output = http.getString();
      if (httpCode == -11)
        Serial.println("Http Code -11");
      Serial.println(output);
      Serial.println("Post successfully send!");
    }

    else {
      Serial.println("Error on HTTP request");
      Serial.println(httpCode);
    }
    http.end();
  }
}

boolean connectWifi() {

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wifi");
    smartDelay( TIMEINTERVALL);
    return true;
  }
  else {
    WiFi.begin(ssid, password);
    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i < 5) {
      Serial.println("Connecting to WiFi...");
      smartDelay(TIMEINTERVALL / 5);
      i++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("No connection");
      WiFi.mode( WIFI_MODE_NULL );
      return false;
    }
    return true;
  }
}
String toJSON(DATA d) {
  //Serial.println(d.humidity);
  String s = "{";
  s += String("\"pm25\":\"") +  d.Pm25 + "\",";
  s += String("\"pm10\":\"") +  d.Pm10 + "\",";
  s += String("\"temperature\":\"") +  d.temperature + "\",";
  s += String("\"pressure\":\"") +  d.pressure + "\",";
  s += String("\"humidity\":\"") + d.humidity  + "\",";
  s += String("\"latitude\":\"") +  float2str(d.GPS_latitude, 6) + "\",";
  s += String("\"longitude\":\"") +  float2str(d.GPS_longitude, 6) + "\",";
  s += String("\"altitude\":\"") +  d.GPS_altitude + "\",";
  s += String("\"speed\":\"") +  d.GPS_speed + "\",";
  s += String("\"accX\":\"") +  d.accX + "\",";
  s += String("\"accY\":\"") +  d.accY + "\",";
  s += String("\"accZ\":\"") +  d.accZ + "\",";
  s += String("\"timestamp\":\"") +  d.timestamp;
  s += String("\"}") ;
  //Serial.println(s);
  return s;
}
void updateDisplay(DATA d) {
  display.setRotation(1);                // Display um 90° drehen
  display.setTextColor(GxEPD_BLACK);     // Schriftfarbe Schwarz
  display.setFont(&FreeMono12pt7b);
  display.fillRect(120, 20, 79, 20, GxEPD_WHITE);
  display.setCursor(120, 35);
  String timest = d.timestamp[8] + d.timestamp[9] + ":" + d.timestamp[8] + d.timestamp[9];
  timest = (sizeof(timest) == 5) ? timest : "";
  Serial.println(timest);
  display.print(timest );
  // Temperatur schreiben
  display.fillRect(0, 80, 98, 20, GxEPD_WHITE); //Xpos,Ypos,box-w,box-h
  display.setCursor(2, 95);
  display.setFont(&FreeMono12pt7b);
  display.print((int) d.Pm10);
  display.fillRect(0, 125, 98, 20, GxEPD_WHITE);
  display.setCursor(2, 140);
  display.print((int) d.temperature);
  display.fillRect(0, 170, 98, 20, GxEPD_WHITE);
  display.setCursor(2, 185);
  display.print((int) d.humidity);
  display.fillRect(101, 80, 98, 20, GxEPD_WHITE);
  display.setCursor(102, 95);
  display.print((int) d.Pm25);
  display.fillRect(101, 125, 98, 20, GxEPD_WHITE);
  display.setCursor(102, 140);
  display.print((int) d.pressure);
  display.fillRect(101, 170, 98, 20, GxEPD_WHITE);
  display.setCursor(102, 185);
  display.print((int) d.GPS_altitude );

  if (connectedToWifi) {
    display.fillRect(194, 2, 5, 5, GxEPD_BLACK);
  }
  else {
    display.fillRect(194, 2, 5, 5, GxEPD_WHITE);
  }

  if (GPS_available) {
    display.fillRect(184, 2, 5, 5, GxEPD_BLACK);
  }
  else {
    display.fillRect(184, 2, 5, 5, GxEPD_WHITE);
  }
  display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}

void setup(void) {
  Serial.begin(9600);


  serialSDS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  I2C.begin(SDA, SCL, 400000);

  status = bme.begin(&I2C);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    //while (1);
  }
  mpu6050.update();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Sds011::Report_mode report_mode;
  if (!sds011.get_data_reporting_mode(report_mode)) {
    Serial.println("Sds011::get_data_reporting_mode() failed");
  }
  if (Sds011::REPORT_ACTIVE != report_mode) {
    Serial.println("Turning on Sds011::REPORT_ACTIVE reporting mode");
    if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
      Serial.println("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
    }
  }
  Serial.println("Start GPS");
  delay(200);
  GPSSerial.begin(9600, SERIAL_8N1, S_RX, S_TX);
  //SerialSDS011.flush();
  GPSSerial.flush();
  display.init();                   // e-Ink Display initialisieren
  display.fillScreen(GxEPD_WHITE);  // Display Weiss füllen
  display.setRotation(1);           // Display um 90° drehen
  display.drawBitmap(gImage_IMG_0001, 0, 0, 200 , 200, GxEPD_BLACK);
  display.fillRect(0, 105, 200, 1, GxEPD_BLACK);
  display.update();
}
String float2str(float f, int n) {
  String s = "12345.67890";
  int i, k, st = 0;
  float teiler;
  double d, e;
  char c;

  s = "";
  d = f;
  for (i = 10000; i > 0; i /= 10) {
    // 10000er 1000er 100er 10er 1er
    k = (int)d / i;
    if ((k > 0) | st) {
      st = 1;
      c = k + 48; // ASCII
      s.concat(c);
    }
    d = d - (k * i);
  }
  if (st == 0) s.concat("0"); // wenigstens 0 ausgeben
  if (n > 0) s.concat("."); // Dezimalpunkt
  teiler = 0.1;
  for (i = 0; i < n; i++) {
    e = d / teiler; // 0.1er 0.01er 0.001er 0.0001er
    k = (int)e;
    c = k + 48; // ASCII
    s.concat(c);
    d = d - (k * teiler);
    teiler = teiler / 10.0;
  }
  return s;
}

void loop() {
  service();

}

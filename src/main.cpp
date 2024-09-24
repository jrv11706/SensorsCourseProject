#include <Arduino.h>
#include <WiFi.h>
#include <UltraSonic.cpp>
#include <GY521.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <ThingSpeak.h>
#include <math.h>

//function declarations:
void testPrint(), demoPrint();
void wifiSetup(String), speakThings(), fsrSetup(), bmpSetup(), gyroSetup(), setReference(), adjustDistance(float angleX, float angleY, float* value);
bool fsrRead();
float complimentFilter(float alti, float dist, float angleX, float angleY);

//object instantiations
WiFiClient client;
UltraSonic ultrasonic(6, 5);
GY521 gyro(0x68);
Adafruit_BMP280 bmp280;

//variables
unsigned long channelNumber = 2428179;
const char * apiKey = "EZS9G0OFPNOS4Z7P";

float distance_climbed, x_angle, y_angle, altitude, distance;
float ground_altitude, ground_distance;
float height, height_prev;
float totalHeight;
unsigned long start_time;
int complete_time;

void setup() {
  Serial.begin(9600);
  Wire.begin(18, 19);

  wifiSetup("ESP32");
  ThingSpeak.begin(client);

  pinMode(LED_BUILTIN, OUTPUT);
  ultrasonic.Setup();
  fsrSetup();
  bmpSetup();
  gyroSetup();

  while (!fsrRead()) {}
  setReference();
  delay(1000);
}

void loop() {
  if (fsrRead()) {
    digitalWrite(LED_BUILTIN, LOW);
    complete_time = ((millis() - start_time) / 1000);
    Serial.printf("completed time: %ld s\n", complete_time);
    speakThings();
    delay(2500);

    while (!fsrRead()) {}
    setReference();
  }

  gyro.read();

  x_angle = gyro.getAngleX(); //degrees
  y_angle = gyro.getAngleY() - abs(gyro.getAngleX()) + 90; //degrees
  altitude = (bmp280.readAltitude() * 3.28); //feet
  distance = (ultrasonic.Read() * 0.0328); //feet
  adjustDistance(x_angle, y_angle, &distance);
  
  height_prev = height;
  height = complimentFilter((altitude - ground_altitude), (distance - ground_distance), x_angle, y_angle);
  if (height > height_prev) {
    totalHeight += (height - height_prev);
  }

  //testPrint();
  demoPrint();
  delay(1000);
}

//function definitions:
void wifiSetup(String ssid) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delayMicroseconds(1);
  }
  Serial.println("Connected to Network");
}

void fsrSetup() {
  pinMode(4, OUTPUT);
  pinMode(7, INPUT);
  digitalWrite(4, HIGH);
}

void bmpSetup() {
  bool status = bmp280.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp280.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);

  }
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void gyroSetup() {
  while (gyro.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }
  gyro.setAccelSensitivity(2);  //  8g
  gyro.setGyroSensitivity(1);   //  500 degrees/s
  gyro.setThrottle();
  
  gyro.gxe = 0.032;
  gyro.gye = -0.039;
  gyro.gze = -0.023;
}

bool fsrRead() {
  return digitalRead(7);
}

void setReference() {
  gyro.read();
  x_angle = gyro.getAngleX(); //degrees
  y_angle = gyro.getAngleY() - abs(gyro.getAngleX()) + 90; //degrees
  altitude = (bmp280.readAltitude() * 3.28); //feet
  distance = (ultrasonic.Read() * 0.0328); //feet
  adjustDistance(x_angle, y_angle, &distance);

  ground_altitude = altitude;
  ground_distance = distance;
  
  totalHeight = 0.00;
  height = 0.00;

  start_time = millis();
  digitalWrite(LED_BUILTIN, HIGH);
}

void adjustDistance(float angleX, float angleY, float *value) {
  angleX = angleX * (6.28 / 360);
  angleY = angleY * (6.28 / 360);
  *value = (*value * cos(angleX)) * (cos(angleY));
}

float complimentFilter(float alti, float dist, float angleX, float angleY) {
  float totalAngle = abs(angleX) + abs(angleY);
  if (totalAngle > 35.0) {
    totalAngle = 35.0;
  }
  float bmpWeight = (totalAngle + 15) / 50;
  return ((1 - bmpWeight) * dist) + (bmpWeight * alti);
}

void speakThings() {
  ThingSpeak.setField(1, totalHeight);
  ThingSpeak.setField(2, complete_time);
  ThingSpeak.writeFields(channelNumber, apiKey);
}

void testPrint() {
  Serial.printf("distance: %0.2f ft\n", ultrasonic.Read() * 0.0328);
  Serial.printf("Adjusted distance: %0.2f ft\n", distance); 

  Serial.printf("reference distance: %0.2f ft\n", ground_distance);
  Serial.printf("reference altitude: %0.2f ft\n", ground_altitude);

  Serial.printf("calculated distance: %0.2f ft\n", distance - ground_distance);
  Serial.printf("calculated altitude: %0.2f ft\n", altitude - ground_altitude);
  Serial.printf("Compliment: %0.2f ft\n", height);

  Serial.printf("x angle: %0.2f deg\n", x_angle);
  Serial.printf("y angle: %0.2f deg\n", y_angle);

  Serial.printf("Altitude: %0.2f ft\n", altitude);

  Serial.printf("totalHeight: %0.2f ft\n", totalHeight);

  Serial.printf("Button: %d\n", fsrRead());

  Serial.println();
}

void demoPrint() {
  Serial.printf("filtered height: %0.2f ft\n", height);
  Serial.printf("total height: %0.2f ft\n", totalHeight);
  int seconds = (millis() - start_time) / 1000;
  int minutes = seconds / 60;
  seconds = seconds % 60;
  Serial.printf("time: %d:%ld\n\n", minutes, seconds);
}
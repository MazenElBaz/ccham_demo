// SPDX-FileCopyrightText: 2019 Brent Rubell for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// Adafruit IO - Analog Devices ADT7410 + ADXL343 Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Brent Rubell for Adafruit Industries
// Copyright (c) 2019 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

#define WIFI_SSID "SM-G973W3730"
#define WIFI_PASS "7802983369"

/************************** Configuration ***********************************/
// time between sending data to adafruit io, in seconds.
#define IO_DELAY 1
/************************ Example Starts Here *******************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_ADT7410.h"
#include <Adafruit_ADXL343.h>
#include <cmath>

#include <NTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include "base64.hpp"
#include "CloudIoTCoreDevice.h"
#include "ciotc_config.h" // Update this file with your configuration
#define CLOUD_IOT_CORE_HTTP_HOST "cloudiotdevice.googleapis.com"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
CloudIoTCoreDevice *device;

const int httpsPort = 443;
String apiPath;

float tempC, accelX, accelY, accelZ, accelAVG, baseline, change;

#define num_samples 5

#define threshold_change 50

float baseline_readings[num_samples] = {};

int baseline_index = 0;

bool tap_off = true;

bool startup = true;

int ON_count = 0;

int ON_duration = 0;

bool tap_on = false;

int sample_idx = 0;


float recent_samples[num_samples] = {};

// Create the ADT7410 temperature sensor object
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();

// Create the ADXL343 accelerometer sensor object
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);


void setup()
{

  //Filling samples
  for (int i=0; i<num_samples; i++) {
    recent_samples[i] = 0;
  }
  
  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while (!Serial)
    ;

  Serial.println("Adafruit IO - ADT7410 + ADX343");

  /* Initialise the ADXL343 */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_8_G);

  /* Initialise the ADT7410 */
  if (!tempsensor.begin())
  {
    Serial.println("Couldn't find ADT7410!");
    while (1)
      ;
  }

  // sensor takes 250 ms to get first readings
  delay(250);

  ESP.wdtDisable();

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
  device = new CloudIoTCoreDevice(
    project_id, location, registry_id, device_id,
    private_key_str);
  apiPath = device->getSendTelemetryPath();
  timeClient.update();

}


void sendToCloud(unsigned char string[]) {
  WiFiClientSecure wifi;
  wifi.setInsecure();//skip verification
  long long int timeNow = timeClient.getEpochTime();
  
  wdt_reset();
  wdt_disable();
  String jwt = device->createJWT(timeNow, jwt_exp_secs);
  wdt_enable(0);
  wdt_reset();
  unsigned char base64[200];
  // encode_base64() places a null terminator automatically, because the output is a string
  
  int base64_length = encode_base64(string, strlen((char *) string), base64);
  String body = String("{\"binary_data\":\"") + (char *) base64 + "\"}";

  if (!wifi.connect(CLOUD_IOT_CORE_HTTP_HOST, httpsPort)) {
    Serial.println("Connection failed!");
  }
  else {
    Serial.println("Connected to server!");
    wifi.println("POST " + apiPath + " HTTP/1.1");
    wifi.println(String("host: ") + CLOUD_IOT_CORE_HTTP_HOST);
    wifi.println("authorization: " + String("Bearer ") + jwt);
    wifi.println("content-type: application/json");
    wifi.println("Connection: close");
    wifi.println("cache-control: no-cache");
    wifi.print("Content-Length: ");
    wifi.println(body.length());
    wifi.println();
    wifi.println(body);


    while (wifi.connected()) {
      String line = wifi.readStringUntil('\n');
      if (line == "\r") {
        Serial.println("headers received");
        break;
      }
    }

    while (wifi.available()) {
      char c = wifi.read();
      Serial.write(c);
    }
    wifi.stop();
  }

}

float get_avg(float* recent_samples, int len) {
  float avg = 0.0;
  for (int i=0; i<len; i++) {
    avg += recent_samples[i];
  }
  avg /= len;
  return avg;
}


void loop()
{
  timeClient.update();


   /* Get a new accel. sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  //accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  //accelZ = event.acceleration.z;
  //accelAVG = (accelX + accelY + accelZ)/3;
  if(tap_off == true && startup == true){
    baseline_readings[baseline_index] = accelY;
    baseline_index++;

    if(baseline_index == num_samples){
      baseline = get_avg(baseline_readings, num_samples);
      startup = false;
    }
  }


  if(baseline_index == num_samples){
    change = abs(abs(accelY-baseline)/baseline)*100;

 
    recent_samples[sample_idx] = change;
    sample_idx = (sample_idx + 1) % num_samples;
    float recent_avg = get_avg(recent_samples, num_samples);
       Serial.print("recent_avg: ");
    Serial.println(recent_avg);
    
    if(recent_avg > threshold_change){
      Serial.println("TAP IS ON");
      tap_on = true;
      tap_off = false;
      ON_count++;
    }
    else{
      Serial.println("TAP IS OFF");
      tap_on = false;
      tap_off = true;

      if(ON_count > 0){
          Serial.println(ON_count);
        ON_duration = ON_count * 0.25;
        ON_count = 0;

        if(ON_duration >= 3 && ON_duration < 15){
          Serial.println("Filling up water bottle");
          sendToCloud((unsigned char *) "Filling up water bottle");
        }
        else if(ON_duration >= 15 && ON_duration < 25){
          Serial.println("Washing hands or filling up a cooking pot");
          sendToCloud((unsigned char *)"Washing hands or filling up a cooking pot");
        }
        else if(ON_duration >= 25){
          Serial.println("Washing dishes");
          sendToCloud((unsigned char *)"Washing dishes");
        }


      }
    }
  }


  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("X: "); Serial.print(accelX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelY); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(accelZ); Serial.print("  ");Serial.println("m/s^2 ");
  
  // Read and print out the temperature
//  tempC = tempsensor.readTempC();
//  Serial.print("Temperature: "); Serial.print(tempC); Serial.println("C");

  for (int i = 0; i < IO_DELAY; i++)
  {
    delay(250);
    wdt_reset();
  }
}

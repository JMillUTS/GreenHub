#include <Wire.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// Pin definitions
const int soilSensorPin = A0;
const int pumpRelayPin = 2;
const int growLampRelayPin = 3;
const int sensorPowerPin = 6;
const int dht11Pin = 7;
const int soilThreshold = 100;  // Adjust according to your desired soil moisture threshold
SoftwareSerial EspSerial(9, 10);

// Timings
const unsigned long growLampOnDuration = 0.01 * 60 * 60 * 1000UL;  // 1 min in milliseconds
const unsigned long growLampOffDuration = 0.02 * 60 * 60 * 1000UL - growLampOnDuration;
unsigned long growLampTimer = 0;
const unsigned long sensorCheckDuration = 2 * 1000UL;               // 2 seconds in milliseconds
const unsigned long sensorCheckInterval = 0.01 * 60 * 60 * 1000UL;  // 12 hours in milliseconds
unsigned long sensorCheckTimer = 0;
const unsigned long pumpDuration = 6 * 1000UL;  // 6 seconds in milliseconds
unsigned long pumpTimer = 0;
const unsigned long dht11CheckInterval = 0.5 * 60 * 1000UL;  // 0.5 minute in milliseconds
unsigned long dht11CheckTimer = 0;

//Variables
String str;

// Initialize DHT11 sensor
DHT dht11(dht11Pin, DHT11);

void setup() {
  Serial.begin(9600);
  EspSerial.begin(9600);
  pinMode(soilSensorPin, INPUT);
  //pinMode(pumpRelayPin, OUTPUT);
  pinMode(growLampRelayPin, OUTPUT);
  pinMode(sensorPowerPin, OUTPUT);
  dht11.begin();

  // Begin serial feed
  Serial.println("Greenhouse System");
  // Turn grow lamp on
  digitalWrite(growLampRelayPin, HIGH);
  growLampTimer = millis();
}

void loop() {
  unsigned long currentTime = millis();
  int soilMoisture = 0;
  // Check soil moisture every 12 hours
  if (currentTime - sensorCheckTimer >= sensorCheckInterval) {
    digitalWrite(sensorPowerPin, HIGH);
    delay(sensorCheckDuration);
    soilMoisture = analogRead(soilSensorPin);
    digitalWrite(sensorPowerPin, LOW);
    //reset sensor check timer
    sensorCheckTimer = currentTime;
    bool needsWatering = soilMoisture < soilThreshold;
    if (needsWatering) {
      digitalWrite(pumpRelayPin, HIGH);
      pumpTimer = currentTime;
      delay(pumpDuration);
      digitalWrite(pumpRelayPin, LOW);
      Serial.println("Please water");
    }
    Serial.print("Moisture: ");
    Serial.print(soilMoisture);
    Serial.println(" units");
    Serial.println(needsWatering ? "Watering..." : "Healthy");
  }
  // Check temperature and humidity every minute
  if (currentTime - dht11CheckTimer >= dht11CheckInterval) {
    float humidity = dht11.readHumidity();
    float temperature = dht11.readTemperature();

    // Display temp

    str = String("T: ")+String(temperature)+String(" degrees C, Humidity: ")+String(humidity)+String("%");
    Serial.println(str);
    EspSerial.println(str);
    dht11CheckTimer = currentTime;
  }

  // Control grow lamp: on for 17 hours, off for 7 hours
  if (currentTime - growLampTimer >= growLampOnDuration && digitalRead(growLampRelayPin) == HIGH) {
    digitalWrite(growLampRelayPin, LOW);
  } else if (currentTime - growLampTimer >= growLampOffDuration && digitalRead(growLampRelayPin) == LOW) {
    digitalWrite(growLampRelayPin, HIGH);
    growLampTimer = currentTime;
  }
}
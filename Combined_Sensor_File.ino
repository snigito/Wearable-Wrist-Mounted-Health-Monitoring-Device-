#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include "Adafruit_BME680.h" 
#include <Adafruit_MLX90614.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "SSD1306Wire.h"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

#define API_KEY "your-api-key"
#define DATABASE_URL "your-db-url"

const char* ssid = "your-ssid";
const char* password = "your-password";

Adafruit_BME680 bme;
Adafruit_MLX90614 mlx;
bioData body;  

SSD1306Wire display(0x3C, 17, 18, GEOMETRY_128_64, I2C_TWO);

int counter = 0;
int networkCounter = 0;
const int GSR_PIN = 1; // Analog input pin for GSR sensor

unsigned long lastMLXRead = 0;
float skinTemp = NAN;
const unsigned long mlxInterval = 1000;

int resPin = 4;
int mfioPin = 13;
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void displayReset(void) {
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, HIGH);
  delay(1);
  digitalWrite(RST_OLED, LOW);
  delay(1);
  digitalWrite(RST_OLED, HIGH);
  delay(1);
}

void setup() {
  Wire.begin(40, 39); // SDA = GPIO 40, SCL = GPIO 39 
  delay(100);
  Serial.begin(115200);
  //Wifi intialization
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = "0qw1wckJo4rQgCu6c99IDwZQYdQtIazrAu6xUeLC";  
  
  //Firebase Initalization
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize sensors
  Serial.println("Initializing sensors");
  int result = bioHub.begin();
  if (result != 0) Serial.println("Could not communicate with the PulseOx sensor!");
  int error = bioHub.configBpm(MODE_ONE);
  if (error != 0) Serial.println("Error configuring PulseOx sensor.");

  if(error != 0) {
    Serial.println("Error configuring PulseOx sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  if (!bme.begin()){
    Serial.println("Could not find BME680 sensor. Check wiring!");
    while (1);
  }

  delay(50);

  if (!mlx.begin()) {  // 0x5A is the default MLX90614 address
    Serial.println("Error connecting to MLX90614 sensor. Check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  VextON();
  displayReset();
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  counter = 0;
  networkCounter = 0;
}

void loop() {
  //BPM sensing 
  body = bioHub.readBpm();

  //Temp sensing
  unsigned long now = millis();
  if (now - lastMLXRead >= mlxInterval) {
    skinTemp = mlx.readObjectTempC();
    lastMLXRead = now;
  }
  
  //GSR sensor 
  double gsrValue = analogRead(GSR_PIN);

  if(counter >= 100){
    if (bme.performReading()) {
      double heartRate = body.heartRate;
      double confidence = body.confidence;
      double oxygen = body.oxygen;
      int status = body.status;
      double bmeTemp = bme.temperature;
      double bmeHum = bme.humidity;
      double pressure = bme.pressure / 100.0;
      double gasResistance = bme.gas_resistance / 1000.0;

      //Serial Output
      Serial.print(heartRate); 
      Serial.print(",");
      Serial.print(confidence); 
      Serial.print(",");
      Serial.print(oxygen); 
      Serial.print(",");
      Serial.print(status);   
      Serial.print(",");
      Serial.print(gsrValue);
      Serial.print(",");
      Serial.print(bmeTemp);
      Serial.print(",");
      Serial.print(bmeHum);
      Serial.print(",");
      Serial.print(pressure);
      Serial.print(",");
      Serial.print(gasResistance);
      Serial.print(",");
      Serial.print(skinTemp);
      
      delay(500);

      //OLED display
      display.clear();
      display.drawString(0, 0, "Heart Rate: " + String(heartRate, 1) + " BPM");
      display.drawString(0, 10, "SpO2: " + String(oxygen, 1) + "%");
      display.drawString(0, 20, "Skin Temperature: " + String(skinTemp, 1) + " C");
      display.drawString(0, 30, "GSR: " + String(gsrValue, 1));
      display.drawString(0, 40, "Ambient Temperature: " + String(bmeTemp, 1));
      display.drawString(0, 50, "Humidity: " + String(bmeHum, 1));
      display.display();
    }

    Serial.println();

    counter = 0;
  } else {
    counter++;
  }

  if(networkCounter >= 1000){
    if (body.confidence < 85 && body.confidence != 0) {
      Serial.println("Low confidence reading — skipping upload.");
      networkCounter = 0; 
      return;
    }
    FirebaseJson json;

    //JSON for Wifi Upload
    json.set("heartRate", body.heartRate);
    json.set("confidence", body.confidence);
    json.set("oxygen", body.oxygen);
    json.set("status", body.status);
    json.set("gsr", gsrValue);
    json.set("temp", bme.temperature);
    json.set("humidity", bme.humidity);
    json.set("pressure", bme.pressure / 100.0);
    json.set("gas", bme.gas_resistance / 1000.0);
    json.set("skinTemp", skinTemp);

    if (Firebase.setJSON(fbdo, "/sensorData", json)) {
      Serial.println("✅ JSON sent to Firebase");
    } else {
      Serial.print("❌ Firebase error: ");
      Serial.println(fbdo.errorReason());
    }
    networkCounter = 0;
  } else {
    networkCounter++;
  }  
}


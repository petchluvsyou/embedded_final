// Include required libraries
#include <DHT.h>
#include <HardwareSerial.h>

// Define pins for DHT22 and GP2Y10
#define DHTPIN PB6     // Pin connected to the DHT22 data pin
#define DHTTYPE DHT22  // DHT 22 (AM2302)
#define GP2Y10_LED_PIN PA6   // Pin connected to the LED control pin of GP2Y10
#define GP2Y10_SIGNAL_PIN PA1 // Pin connected to the analog signal output of GP2Y10
HardwareSerial Serial1(PA10, PA9);
// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial1.begin(115200);
  // Initialize DHT sensor
  dht.begin();
  // Initialize pins
  pinMode(GP2Y10_LED_PIN, OUTPUT);
  pinMode(GP2Y10_SIGNAL_PIN, INPUT);
}

void loop() {
  // Read data from DHT22 sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again)
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read data from GP2Y10 sensor
  digitalWrite(GP2Y10_LED_PIN, LOW);  // Turn on the LED
  delayMicroseconds(280);             // Wait for the sensor to stabilize

  int sensorValue = analogRead(GP2Y10_SIGNAL_PIN); // Read analog value
  digitalWrite(GP2Y10_LED_PIN, HIGH); // Turn off the LED
  delayMicroseconds(40);              // Wait before the next reading
  
  // Convert the sensor value to voltage
  float voltage = sensorValue * (5.0 / 1024.0);

  // Calculate dust density in mg/m3
  float dustDensity = max(170.0 * voltage - 100.0, 0.0);

  // Print the results to the Serial Monitor
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C\t");
  Serial.print("Dust Density: ");
  Serial.print(dustDensity);
  Serial.println(" ug/m3");
  String strh = String(humidity, 2);
  String strt = String(temperature, 2);
  String strd = String(dustDensity, 2);
  String result = strt+" "+strh+" "+strd;
  Serial1.println(result);
  Serial1.flush();

  // Wait before the next loop iteration
  delay(1000);
}

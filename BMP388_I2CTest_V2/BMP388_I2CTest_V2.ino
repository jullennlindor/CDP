#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// Create a BMP3XX sensor object
Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

 // Initialize I2C communication on the secondary bus (Wire1) with custom pins
Wire1.begin();

if (!bmp.begin_I2C(0x77, &Wire1)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
}

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  // Perform sensor reading
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Print temperature data
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  // Print pressure data
  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  // Print altitude data
  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  // Wait 2 seconds before the next reading
  Serial.println();
  delay(2000);
}
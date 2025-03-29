#include <Wire.h>              // For I2C communication
#include <Adafruit_Sensor.h>   // Adafruit sensor library
#include "Adafruit_BMP3XX.h"   // Library for BMP388 pressure/temperature sensor
#include <SD.h>                // SD card library for reading and writing to SD card

#define SEALEVELPRESSURE_HPA (1013.25)  // Define the sea level pressure in hPa (standard atmospheric pressure)

Adafruit_BMP3XX bmp;           // Create an object for the BMP388 sensor
File dataFile;                 // File object to handle writing data to the SD card
unsigned long startTime;       // Start time to track elapsed time

void setup() {
  // Initialize serial communication for debugging (with baud rate 115200)
  Serial.begin(115200);
  
  // Wait for the serial connection to be established (if connected to a computer)
  while (!Serial);

  // Print a message to the serial monitor to indicate that the setup has started
  Serial.println("Teensy 4.1 BMP388 Unit Test");

  // Initialize the SD card in the Teensy 4.1
  if (!SD.begin(BUILTIN_SDCARD)) { // How do we refer to the SD card in the Teensy 4.1's built-in SD card slot
    // If SD card initialization fails, print an error message and halt execution
    Serial.println("SD card initialization failed!");
    while (1);  // Infinite loop to stop the program in case of failure
  }

  // Delete the existing data file if it exists
  if (SD.exists("BMP388_data.csv")) {
    SD.remove("BMP388_data.csv");  // Remove existing file
    Serial.println("Existing data file deleted.");
  }

  // Create a new CSV file on the SD card to store data
  dataFile = SD.open("BMP388_data.csv", FILE_WRITE);  // Open (or create) a file for writing
  if (dataFile) {
    // Write the header line to the CSV file, fill in the units for each column!
    dataFile.println("timestamp,temperature,pressure,altitude");
  } else {
    // If the file can't be opened, print an error message and halt execution
    Serial.println("Error opening BMP388_data.csv");
    while (1);  // Infinite loop to stop the program
  }

  Wire.begin(); // Initialize I2C communication

  // Initialize the BMP388 sensor with I2C communication
  if (!bmp.begin_I2C()) {  // Default I2C pins for Teensy 4.1 are SDA (pin 18) and SCL (pin 19)
    // If sensor initialization fails, print an error message and halt execution
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    while (1);  // Infinite loop to stop the program
  }

  // Configure BMP388 sensor settings for data acquisition
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Set temperature oversampling to 8x for higher precision
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Set pressure oversampling to 4x
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Set the IIR filter to reduce noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                  // Set output data rate

  startTime = millis();  // Initialize startTime when the setup is completed

  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.

  Serial.println("jullenn");  // Print to serial monitor that data is starting to be collected

}

void loop() {
  // Check if 1 minute has passed
  if (millis() - startTime >= 60000) { // Compare current milliseconds to 1 minute in milliseconds
    dataFile.close();  // Close the file 
    Serial.println("1 minute has passed. Stopping the data logging.");
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off
    while(1);  // Stop the program after data logging completes
  }

  // Verify SD can be accessed and log data to it
  if (SD.mediaPresent()) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by an amount to get seconds

    dataFile.print(timestamp);               // Write the timestamp to the file
    dataFile.print(",");                     // CSV delimiter (comma)

    // Take a reading from the BMP388 sensor
    if (bmp.performReading()) {
      // If reading can be  taken, write the sensor data to the SD card in CSV format
      dataFile.print(bmp.temperature);         // Write the temperature in Celsius
      dataFile.print(",");                     // CSV delimiter
      dataFile.print(bmp.pressure / 100.0);    // Write the pressure in hPa (Pa to hPa conversion)
      dataFile.print(",");                     // CSV delimiter
      dataFile.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters
    } else {
      // If reading fails, print an error message and halt the program
      Serial.println("Failed to read BMP Sensor!");
      while(1);
    }

  } else {
    // If the file can't be accessed, print an error
    Serial.println("Error writing to BMP388_data.csv");
    while(1);
  }

  digitalWrite(LED_BUILTIN, HIGH);  // turn the Teensy LED on

  // Wait for 1 second before starting loop over and taking another reading
  delay(1000);  // how many milliseconds in 1 second?
}

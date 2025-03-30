#include <Wire.h>              // For I2C communication
#include <Adafruit_Sensor.h>   // Adafruit sensor library
#include <Adafruit_BMP3XX.h>   // Library for BMP388 pressure/temperature sensor
#include <SD.h>                // SD card library for reading and writing to SD card

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard atmospheric pressure at sea level in hPa

Adafruit_BMP3XX bmp;           // Create an object for the BMP388 sensor
File dataFile;                 // File object to handle writing data to the SD card
unsigned long startTime;       // Start time to track elapsed time

// RGB LED pin definitions
const int redPin = 3; // Define pin on Teensy for Red
const int greenPin = 4; // Define pin on Teensy for Green
const int bluePin = 5; // Define pin on Teensy for Blue

void setup() {
  // Initialize serial communication for debugging (with baud rate 115200)
  Serial.begin(115200);
  
  // Wait for the serial connection to be established (if connected to a computer)
  while (!Serial);

  // Print a message to the serial monitor to indicate that the setup has started
  Serial.println("Teensy 4.1 BMP388 Unit LED Test");

  // initialize the digitals pin as an outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Turn the LED off initially
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Initialize the SD card in the Teensy 4.1's built-in SD card slot
  if (!SD.begin(BUILTIN_SDCARD)) {  // Teensy 4.1 has a built-in SD card reader
    // If SD card initialization fails, print an error message and halt execution
    Serial.println("SD card initialization failed!");
    digitalWrite(redPin, HIGH);  // Turn on red LED for SD card failure
    while (1);  // Infinite loop to stop the program
  }

  // Delete the existing data file if it exists
  if (SD.exists("BMP388_data.csv")) {
    SD.remove("BMP388_data.csv");  // Remove existing file
    Serial.println("Existing data file deleted.");
  }

  // Create a new CSV file on the SD card to store data
  dataFile = SD.open("BMP388_data.csv", FILE_WRITE);  // Open (or create) a file for writing
  if (dataFile) {
    // Write the header line to the CSV file (labels for the columns)
    dataFile.println("Timestamp,Temperature (C),Pressure (hPa),Altitude (m)");
  } else {
    // If the file can't be opened, print an error message and halt execution
    Serial.println("Error opening BMP388_data.csv");
    digitalWrite(redPin, HIGH); // Turn the red pin on 
    while (1);  // Infinite loop to stop the program
  }

  Wire.begin(); // Initialize I2C communication

  // Initialize the BMP388 sensor with I2C communication
  if (!bmp.begin_I2C()) {  // Default I2C pins for Teensy 4.1 are SDA (pin 18) and SCL (pin 19)
    // If sensor initialization fails, print an error message and halt execution
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    digitalWrite(redPin, HIGH); // Turn the red pin on 
    while (1);  // Infinite loop to stop the program
  }

  // Configure BMP388 sensor settings for data acquisition
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Set temperature oversampling to 8x for higher precision
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Set pressure oversampling to 4x
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Set the IIR filter to reduce noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                  // Set output data rate to 1 Hz (1 sample per second)

  startTime = millis();  // Initialize startTime when the setup is completed

  pinMode(LED_BUILTIN, OUTPUT);   // initialize digital pin LED_BUILTIN as an output.

  Serial.println("Data Collection Starting...");  // SD card initialized successfully

}

void loop() {
  // Check if 1 minute has passed
  if (millis() - startTime >= 60000) { // 1 minute in milliseconds
    dataFile.close();  // Close the file after 1 minute of data logging
    Serial.println("1 minute has passed. Stopping the data logging.");
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (HIGH is the voltage level)
    digitalWrite(redPin, LOW);  // Turn off red LED
    digitalWrite(greenPin, LOW);  // Turn off green LED
    digitalWrite(bluePin, HIGH);   // Turn on blue LED
    while(1);  // Stop the program after data logging completes
  }

  // Verify SD can be accessed and log data to it
  if (SD.mediaPresent()) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by 1000 to get seconds

    dataFile.print(timestamp);               // Write the timestamp
    dataFile.print(",");                     // CSV delimiter (comma)

    // Attempt to take a reading from the BMP388 sensor
    if (bmp.performReading()) {
      // If reading can be  taken, write the sensor data to the SD card in CSV format
      dataFile.print(bmp.temperature);         // Write the temperature in Celsius
      dataFile.print(",");                     // CSV delimiter
      dataFile.print(bmp.pressure / 100.0);    // Write the pressure in hPa (Pa to hPa conversion)
      dataFile.print(",");                     // CSV delimiter
      dataFile.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters
      digitalWrite(redPin, LOW);    // Turn off red LED
      digitalWrite(greenPin, HIGH);   // Turn on green LED for successful logging
    } else {
      // If reading fails, print an error message and halt the program
      Serial.println("Failed to read BMP Sensor!");
      digitalWrite(greenPin, LOW); // Ensure green LED is off
      digitalWrite(redPin, HIGH); // Make sure red LED is on
      while(1); // Program halts
    }

  } else {
    // If the file can't be accessed, print an error
    Serial.println("Error writing to BMP388_data.csv");
    digitalWrite(greenPin, LOW); // turn off green LED
    digitalWrite(redPin, HIGH); // Turn on red LED
    while(1); // Program halts
  }

  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

  // Wait for 1 second before taking another reading
  delay(1000);  // 1 second delay (1000 milliseconds)
}

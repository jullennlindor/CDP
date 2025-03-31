// Include Libraries
#include <Wire.h>              // I2C communication
#include <Adafruit_Sensor.h>   // Adafruit sensor
#include <Adafruit_BMP3XX.h>   // BMP388 pressure/temperature sensor
#include <LIS3MDL.h>    // LIS3MDL Sensor
#include <LSM6.h>       // LSM6 Sensor
#include <SD.h>                // SD card

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard atmospheric pressure at sea level in hPa

// Define Objects
Adafruit_BMP3XX bmp;           // Create an object for the BMP388 sensor
LIS3MDL mag;                    // Object to handle magnetometer sensor
LSM6 imu;                       // Object to handle IMU (Inertial Measurement Unit) sensor
File dataFile;                  // SD file object to handle writing data to SD card
File logFile;                    // Log file for status messages
unsigned long startTime;        // Start time to track elapsed time

// Min and Max calibration values for the magnetometer
LIS3MDL::vector<int16_t> m_min = {-32767, -32767, -32767}; 
LIS3MDL::vector<int16_t> m_max = {+32767, +32767, +32767}; 

// RGB LED pin definitions
const int redPin = 3;
const int greenPin = 4;
const int bluePin = 5;

void setup() {

  

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Initialize the SD card in the Teensy 4.1's built-in SD card slot
  if (!SD.begin(BUILTIN_SDCARD)) {  // Teensy 4.1 has a built-in SD card reader
    // If SD card initialization fails, print an error message and halt execution
    digitalWrite(redPin, HIGH);  // Turn on red LED for SD card failure
    
    while (1);  // Infinite loop to stop the program
  }

  // Delete the existing data file if it exists
  if (SD.exists("Log.txt")) {
    SD.remove("Log.txt");  // Remove existing file
  }

  logFile = SD.open("Log.txt", FILE_WRITE);
  logFile.println("New log file created.");

 

  // Delete the existing data file if it exists
  if (SD.exists("Flight.csv")) {
    SD.remove("Flight.csv");  // Remove existing file
    logFile.println("Existing data file deleted.");
  }

  // Create a new CSV file on the SD card to store data
  dataFile = SD.open("Flight.csv", FILE_WRITE);  // Open (or create) a file for writing
  if (dataFile) {
    // Write the header line to the CSV file (labels for the columns)
    dataFile.println("Timestamp (s),Temperature (C),Pressure (hPa),Altitude (m),Heading (deg),Pitch (deg),Roll (deg),Yaw (deg)");
  } else {
    // If the file can't be opened, print an error message and halt execution
    logFile.println("Error opening Flight.csv on startup!");
    digitalWrite(redPin, HIGH);
    while (1);  // Infinite loop to stop the program
  }

  Wire1.begin(); // Initialize I2C communication over second pair of I2C Teensy pins

  // Initialize the BMP388 sensor with I2C communication
  if (!bmp.begin_I2C(0x77, &Wire1)) {  // Uses second pair of I2C pins for Teensy 4.1
    // If sensor initialization fails, print an error message and halt execution
    digitalWrite(redPin, HIGH);
    while (1);  // Infinite loop to stop the program
  }

  // Set oversampling and filter settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

  Wire.begin(); // Initialize I2C communication for IMU
  
  // Initialize IMU (LIS3MDL and LSM6)
  if (!imu.init() || !mag.init()) {
    digitalWrite(redPin, HIGH);
    while (1); // Halt if IMU initialization fails
  }

  // Default LIS3MDL and LSM6 settings
  mag.enableDefault(); // Enable default settings for the magnetometer
  imu.enableDefault();

  startTime = millis();

  logFile.println("Setup completed, data collection starting...");
  


  logFile.flush();
}

void loop() {
  // Check if 20 minutes has passed
  if (millis() - startTime >= 1200000) { // 20 minutes in milliseconds
    dataFile.close();
    logFile.println("20 minutes has passed. Stopping the data logging.");
    logFile.close();
    digitalWrite(redPin, LOW);  // Turn off red LED
    digitalWrite(greenPin, LOW);  // Turn off green LED
    digitalWrite(bluePin, HIGH);   // Turn on blue LED
    while(1);  // Stop the program after data logging completes
  }

  // Verify Datalog file is accessible
  if (SD.mediaPresent()) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by 1000 to get seconds
    
    dataFile.print(timestamp);               // Write the timestamp
    dataFile.print(",");

    // Take a reading from the BMP388 sensor
    if (bmp.performReading()) {
      dataFile.print(bmp.temperature);
      dataFile.print(",");   
      dataFile.print(bmp.pressure / 100.0);
      dataFile.print(",");   
      dataFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      dataFile.print(",");   
    } else {
      // If reading fails, print an error message and proceed
      logFile.println("Failed to read BMP388!");
      char temperature = '-';
      char pressure = '-';
      char altitude = '-';
      dataFile.print(temperature);
      dataFile.print(",");   
      dataFile.print(pressure);
      dataFile.print(",");   
      dataFile.print(altitude);
      dataFile.print(",");   
    }

    // Take a reading from the IMU sensor
    if (imu.init() && mag.init()) {
      // Perform Magnetometer and IMU Readings
      mag.read();
      imu.read();
      // Calculate heading, pitch, and roll angles from sensor data
      float heading = computeHeading();
      float pitch = computePitch();
      float roll = computeRoll();
      float yaw = computeYaw();
      dataFile.print(heading);
      dataFile.print(",");   
      dataFile.print(pitch);
      dataFile.print(",");   
      dataFile.print(roll);
      dataFile.print(",");   
      dataFile.println(yaw);
    } else {
      logFile.println("Failed to read IMU!");
      char heading = '-';
      char pitch = '-';
      char roll = '-';
      char yaw = '-';
      dataFile.print(heading);
      dataFile.print(",");   
      dataFile.print(pitch);
      dataFile.print(",");   
      dataFile.print(roll);
      dataFile.print(",");   
      dataFile.println(yaw);
    }

   dataFile.flush(); // Flush data to SD after every complete reading

    // Turn status LED green if all sensors are connected
    if (bmp.performReading() && imu.init() && mag.init()) {
     digitalWrite(redPin, LOW);    // Turn off red LED
     digitalWrite(greenPin, HIGH);   // Turn on green LED to make yellow
    }

    // Turn status LED yellow if one of the sensors is not connected
    if (!bmp.performReading() || !imu.init() || !mag.init()) {
     digitalWrite(redPin, HIGH);    // Turn on red LED to make yellow
     digitalWrite(greenPin, HIGH);   // Turn on green LED to make yellow
    }

    // Turn status LED red if none of the sensors is not connected
    if ( !bmp.performReading() && !imu.init() && !mag.init()) {
     digitalWrite(redPin, HIGH);    // Turn on red LED to 
     digitalWrite(greenPin, LOW);   // Turn off green LED
    }

  } else {
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, HIGH);
    // Start retry attempts
    int retryCount = 0; // Create integer to track retries 
    const int maxRetries = 10; // Set max retry number
    // Enter while loop for up to 10 retries
    while (retryCount < maxRetries ) {
      // Try to reconnect to the SD card 
      if (SD.begin(BUILTIN_SDCARD)) {
        break; // exits the while loop if successful
      } else {
      retryCount++; //increase count by 1
      delay(500); // wait 0.5 seconds before trying again
      }
    }
    // Stop code once max retry attempts reach
    if (retryCount == 10) {
      while(1);
    }
    logFile.println("SD connection severed and then recovered."); 
    logFile.print("Retry Count:");
    logFile.println(retryCount);
    logFile.println("Data collection resuming...");
    logFile.flush(); 
  }

  delay(1000);

}

//Functions Below... Outside the scope of this project

// Compute the heading
template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

float computeHeading()
{
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}

// Compute the pitch angle (forward/backward tilt)
float computePitch()
{
  // Convert accelerometer readings to g-force
  float ax = imu.a.x / 32768.0 * 2.0;  // Assuming +/-2g sensitivity
  float ay = imu.a.y / 32768.0 * 2.0;
  float az = imu.a.z / 32768.0 * 2.0;

  // Calculate pitch in degrees
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  return pitch;
}

// Compute the roll angle (side-to-side tilt)
float computeRoll()
{
  // Convert accelerometer readings to g-force
  float ay = imu.a.y / 32768.0 * 2.0;  // Assuming +/-2g sensitivity
  float az = imu.a.z / 32768.0 * 2.0;

  // Calculate roll in degrees
  float roll = atan2(ay, az) * 180 / PI;
  return roll;
}

// Compute the yaw angle (rotation around vertical axis)
float computeYaw()
{
  // Convert gyroscope readings to degrees/second
  float gx = imu.g.x / 32768.0 * 250.0;  // Assuming +/-250dps sensitivity

  // Integrate gyroscope data to estimate yaw angle
  static float yaw = 0;
  static unsigned long lastUpdate = millis();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdate) / 1000.0;  // Convert to seconds
  yaw += gx * deltaTime;
  lastUpdate = currentTime;

  return yaw;
}
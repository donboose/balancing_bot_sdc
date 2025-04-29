#include <Wire.h>
#include <Kalman.h> // Kalman Filter Library
#include <AccelStepper.h>

// Define I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Motor pins
#define dirPin1 32
#define stepPin1 33
#define dirPin2 25
#define stepPin2 26
#define motorInterfaceType 1

// MPU6050 address
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float pitch;

// Timing variables
unsigned long timer = 0;
float dt;
unsigned long loopTimer = 0;
const float LOOP_FREQUENCY = 200.0; // Hz
const float LOOP_INTERVAL = 1000000.0 / LOOP_FREQUENCY; // microseconds

// Motor variables
AccelStepper stepperLeft(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepperRight(motorInterfaceType, stepPin2, dirPin2);
float maxSpeed = 8000.0;
float maxAcceleration = 80000.0;
volatile float currentSpeed = 0.0;

// PID variables
float setpoint = 0.0; // Desired angle (upright)
float Kp = 20.0;  // Start with these values and tune
float Ki = 0.0;
float Kd = 0.0;
float lastError = 0.0;
float integral = 0.0;
float error = 0.0;
float derivative = 0.0;

// Kalman filter
Kalman kalmanY;

// Calibration variables
float gyroYerror = 0.0;
float accYoffset = 0.0;
int calibrationSamples = 1000;

// Safety
const float FALL_ANGLE = 45.0; // Stop motors if robot tilts beyond this angle
bool isFallen = false;
float deadband = 0.2; // Ignores small angle errors to prevent jitter

String inputString = "";        // String to hold incoming serial data
boolean stringComplete = false; // Whether the string is complete

TaskHandle_t motorControlTask;

void setup() {
  Serial.begin(115200);
  inputString.reserve(20);
  printParameters();
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 
  
  // Initialize MPU6050
  initMPU();
  
  // Initialize motor controllers
  stepperLeft.setMaxSpeed(-maxSpeed);
  stepperLeft.setAcceleration(maxAcceleration);
  stepperRight.setMaxSpeed(maxSpeed);
  stepperRight.setAcceleration(maxAcceleration);
  
  Serial.println("Starting MPU6050 calibration...");
  calibrateMPU();
  
  // Initialize timing
  timer = micros();
  loopTimer = micros();
  
  Serial.println("Balance Bot Ready!");

  xTaskCreatePinnedToCore(
    motorControlFunction,  // Function to implement the task
    "MotorControl",        // Name of the task
    10000,                 // Stack size in words
    NULL,                  // Task input parameter
    1,                     // Priority of the task
    &motorControlTask,     // Task handle
    0);                    // Core where the task should run
}

void loop() {
  // Process any incoming serial commands
  processSerialCommands();
  
  // Maintain consistent loop timing
  while (micros() - loopTimer < LOOP_INTERVAL) {
    // Wait until it's time for the next cycle
  }
  
  // Calculate delta time
  dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();
  loopTimer = micros();
  
  // Read the MPU6050 and calculate pitch
  getMPUData();
  calculatePitch();

  // Check if robot has fallen over
  if (abs(pitch) > FALL_ANGLE) {
    if (!isFallen) {
      Serial.println("Robot fallen! Motors stopped.");
      isFallen = true;
    }
    stopMotors();
    return;
  } else {
    isFallen = false;
  }
  
  // PID controller
  error = setpoint - pitch;
  
  // Apply deadband to prevent jitter
  if (abs(error) < deadband) {
    error = 0;
  }
  
  // Integral term with anti-windup
  integral += error * dt;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  
  // Derivative term
  derivative = (error - lastError) / dt;
  lastError = error;
  
  // Calculate PID output
  float output = Kp * error + Ki * integral - Kd * derivative;

  currentSpeed = output;
  
  // Set motor speeds
  stepperLeft.setSpeed(-currentSpeed);
  stepperRight.setSpeed(currentSpeed);
  
  // Print data in formatted output
  static unsigned long lastSerialOutput = 0;
  if (millis() - lastSerialOutput > 100) { // 10Hz output
    Serial.print("time_ms: ");
    Serial.print(millis());
    Serial.print(", pitch: ");
    Serial.print(pitch, 2);
    Serial.print(", error: ");
    Serial.print(error, 2);
    Serial.print(", kp: ");
    Serial.print(Kp, 2);
    Serial.print(", ki: ");
    Serial.print(Ki, 7);
    Serial.print(", kd: ");
    Serial.print(Kd, 2);
    Serial.print(", output: ");
    Serial.print(output, 2);
    Serial.print(", deadband: ");
    Serial.print(deadband, 2);
    Serial.print(", speed: ");
    Serial.println(currentSpeed, 2);
    lastSerialOutput = millis();
  }
}

void initMPU() {
  // Start communication with MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up the MPU-6050
  Wire.endTransmission(true);
  
  // Configure Accelerometer (+/-8g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // +/- 8g full scale range
  Wire.endTransmission(true);
  
  // Configure Gyroscope (500deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // 500 deg/s full scale
  byte error = Wire.endTransmission(true);
  
  if (error != 0) {
    Serial.println("MPU6050 not found!");
    while(1); // Stop execution
  }
  
  // Initialize Kalman filter
  getMPUData();
  float accAngleY = (atan2(AcX, AcZ) * 180 / PI);
  kalmanY.setAngle(accAngleY);
}

void calibrateMPU() {
  float sumGyroY = 0;
  float sumAccY = 0;
  
  Serial.println("Keep the robot stationary for calibration...");
  delay(2000); // Give user time to position the robot
  
  for (int i = 0; i < calibrationSamples; i++) {
    getMPUData();
    sumGyroY += GyY;
    sumAccY += (atan2(AcX, AcZ) * 180 / PI);
    
    if (i % 100 == 0) {
      Serial.print("Calibrating: ");
      Serial.print(i * 100 / calibrationSamples);
      Serial.println("%");
    }
    
    delay(2); // Small delay between readings
  }
  
  gyroYerror = sumGyroY / calibrationSamples;
  accYoffset = sumAccY / calibrationSamples; // This helps find the true vertical
  
  Serial.print("Gyroscope Y-axis bias: ");
  Serial.println(gyroYerror);
  Serial.print("Accelerometer Y-axis offset: ");
  Serial.println(accYoffset);
  Serial.println("Calibration complete!");
}

void getMPUData() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Request 14 registers
  
  // Read registers in one transmission
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Wire.read() << 8 | Wire.read();       // Skip temperature
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void calculatePitch() {
  // Calculate accelerometer angle
  float accAngleY = (atan2(AcX, AcZ) * 180 / PI) - accYoffset;
  
  // Apply Kalman filter to get better angle estimate
  pitch = kalmanY.getAngle(accAngleY, ((GyY - gyroYerror) / 131.0), dt);
}

void stopMotors() {
  currentSpeed = 0;
  stepperLeft.setSpeed(0);
  stepperRight.setSpeed(0);
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}

void serialEvent() {
  // This function is called automatically when serial data is available
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Add character to input string
    if (inChar != '\n' && inChar != '\r') {
      inputString += inChar;
    }
    
    // If newline, set flag to process the command
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processSerialCommands() {
  // Process completed command strings
  if (stringComplete) {
    inputString.trim(); // Remove any whitespace
    
    // Parse commands in format "x:value" where x is a single character
    if (inputString.length() >= 3 && inputString.charAt(1) == ' ') {
      char command = inputString.charAt(0);
      float value = inputString.substring(2).toFloat();
      
      // Process different commands
      switch (command) {
        case 'p': // Change Kp
          Kp = value;
          Serial.print("Kp set to: ");
          Serial.println(Kp);
          break;
          
        case 'i': // Change Ki
          Ki = value;
          Serial.print("Ki set to: ");
          Serial.println(Ki);
          break;
          
        case 'd': // Change Kd
          Kd = value;
          Serial.print("Kd set to: ");
          Serial.println(Kd);
          break;
          
        case 's': // Change max speed
          maxSpeed = value;
          stepperLeft.setMaxSpeed(-maxSpeed);
          stepperRight.setMaxSpeed(maxSpeed);
          Serial.print("Max speed set to: ");
          Serial.println(maxSpeed);
          break;
          
        case 'a': // Change max acceleration
          maxAcceleration = value;
          stepperLeft.setAcceleration(maxAcceleration);
          stepperRight.setAcceleration(maxAcceleration);
          Serial.print("Max acceleration set to: ");
          Serial.println(maxAcceleration);
          break;
          
        case 'o': // Change setpoint offset
          setpoint = value;
          Serial.print("Setpoint set to: ");
          Serial.println(setpoint);
          break;
          
        case 'b': // Change deadband
          deadband = value;
          Serial.print("Deadband set to: ");
          Serial.println(deadband);
          break;
          
        case '?': // Print all parameters
          printParameters();
          break;
          
        default:
          Serial.println("Unknown command. Available commands:");
          Serial.println("p:xx - Set Kp");
          Serial.println("i:xx - Set Ki");
          Serial.println("d:xx - Set Kd");
          Serial.println("s:xx - Set max speed");
          Serial.println("a:xx - Set max acceleration");
          Serial.println("o:xx - Set setpoint offset");
          Serial.println("b:xx - Set deadband");
          Serial.println("?:1  - Print all parameters");
      }
    } else {
      Serial.println("Invalid command format. Use x:value (e.g., p:30)");
    }
    
    // Clear the string and flag for next command
    inputString = "";
    stringComplete = false;
  }
}

void printParameters() {
  Serial.println("\n--- Current Parameters ---");
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.println(Ki);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Max Speed: ");
  Serial.println(maxSpeed);
  Serial.print("Max Acceleration: ");
  Serial.println(maxAcceleration);
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
  Serial.print("Deadband: ");
  Serial.println(deadband);
  Serial.println("------------------------\n");
}

void motorControlFunction(void *pvParameters) {
  for(;;) {
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
    vTaskDelay(1);
  }
}
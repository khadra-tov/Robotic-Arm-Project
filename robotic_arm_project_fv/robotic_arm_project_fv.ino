#include <Wire.h>
#include <Servo.h>
#include <SoftwareWire.h>

const int base_pin = 9;
const int shoulder_pin = 10;
const int elbow_pin = 11;

Servo base_servo;
Servo shoulder_servo;
Servo elbow_servo;

// register addresses
const int MPU_ADDR1 = 0x68;
const int MPU_ADDR2 = 0x69;

const int SDA1_pin = A4;
const int SCL1_pin = A5;
const int SDA2_pin = A2;
const int SCL2_pin = A3;

const int accel_x_reg = 0x3B;
const int accel_y_reg = 0x3D;
const int accel_z_reg = 0x3F;

int16_t ax1,ay1,az1,ax2,ay2,az2;

// default positions
int base_pos = 90;      
int shoulder_pos = 90;  
int elbow_pos = 90;     

// PID control variables
float Kp = 1.3;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0.001;  // Derivative gain
float dt = 0.02; // Time interval (seconds)

float error = 0.0;      // Error term
float prev_error = 0.0;  // Previous error term
float integral = 0.0;   // Integral term
float derivative = 0.0; // Derivative term

SoftwareWire Wire1(SDA1_pin, SCL1_pin);
SoftwareWire Wire2(SDA2_pin, SCL2_pin);

void setup() {
  Wire1.begin();
  Wire2.begin();
  Serial.begin(9600);
  
  base_servo.attach(base_pin);
  shoulder_servo.attach(shoulder_pin);
  elbow_servo.attach(elbow_pin);

  // Calibrate the two MPU6050s
  calibration1();
  calibration2();

  // Move the arm to the default position
  move_arm();
}

void loop() {
  // Read ax and ay from the first MPU6050
  read_accelerations_from_1st_MPU6050();
  // Read az from the second MPU6050
  read_accelerations_from_2nd_MPU6050();
  // Calculate the desired positions of the robotic arm joints based on accelerometer measurements
  servo_positions();

  // PID Controller
  PID_control();

  // Display position and stability measurements on Serial Monitor
  display_measurements();

  // Delay for stability control loop
  delay(dt * 1000);
}

void read_accelerations_from_1st_MPU6050() {
  Wire1.beginTransmission(MPU_ADDR1);
  Wire1.write(accel_x_reg);
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_ADDR1, 6, true);

  ax1 = Wire1.read() << 8 | Wire1.read();
  ay1 = Wire1.read() << 8 | Wire1.read();
  
}

void read_accelerations_from_2nd_MPU6050() {
  Wire2.beginTransmission(MPU_ADDR2);
  Wire2.write(accel_x_reg);
  Wire2.endTransmission(false);
  Wire2.requestFrom(MPU_ADDR2, 6, true);

  ax2 = Wire2.read() << 8 | Wire2.read();
  ay2 = Wire2.read() << 8 | Wire2.read();
  az2 = Wire2.read() << 8 | Wire2.read();
}
void servo_positions() {

  // Calculate desired position for the base servo motor based on accelerometer X-axis
  base_pos = map(ax1, -32768, 32767, 0, 180);

  // Calculate desired position for the shoulder servo motor based on accelerometer Y-axis
  shoulder_pos = map(ay1, -32768, 32767, 0, 180);

  // Calculate desired position for the elbow servo motor based on accelerometer Z-axis
  elbow_pos = map(az2, -32768, 32767, 0, 180);
}

void PID_control() {

  // Control the base servo motor
  error = base_pos - base_servo.read();
  integral += error * dt;
  derivative = (error - prev_error) / dt;
  float control_signal = Kp * error + Ki * integral + Kd * derivative;
  base_servo.write(base_servo.read() + control_signal);

  // Control the shoulder servo motor
  error = shoulder_pos - shoulder_servo.read();
  integral += error * dt;
  derivative = (error - prev_error) / dt;
  control_signal = Kp * error + Ki * integral + Kd * derivative;
  shoulder_servo.write(shoulder_servo.read() + control_signal);

  // Control the elbow servo motor
  error = elbow_pos - elbow_servo.read();
  integral += error * dt;
  derivative = (error - prev_error) / dt;
  control_signal = Kp * error + Ki * integral + Kd * derivative;
  elbow_servo.write(elbow_servo.read() + control_signal);

  // Update the previous error term
  prev_error = error;
}

void display_measurements() {
  Serial.print("Base Position: ");
  Serial.print(base_servo.read());
  Serial.print(", ");
  
  Serial.print("Shoulder Position: ");
  Serial.print(shoulder_servo.read());
  Serial.print(", ");
  
  Serial.print("Elbow Position: ");
  Serial.print(elbow_servo.read());
  Serial.println(", ");
  
  Serial.print("Accelerometer (X1,Y1,Z2): ");
  Serial.print(ax1);
  Serial.print(", ");
  Serial.print(ay1);
  Serial.print(", ");
  Serial.println(az2);

  delay(200);  // Add a delay of 100 milliseconds
}


void move_arm() {
  // Move the arm to the default position
  base_servo.write(base_pos);
  shoulder_servo.write(shoulder_pos);
  elbow_servo.write(elbow_pos);
}
void calibration1() {
  Wire1.beginTransmission(MPU_ADDR1);
  Wire1.write(0x6B);
  Wire1.write(0x80); // reset the MPU6050
  Wire1.endTransmission(true);
  delay(100); 

  Wire1.beginTransmission(MPU_ADDR1);
  Wire1.write(0x6B);
  Wire1.write(0x00); // Wake up from reset
  Wire1.endTransmission(true);
  delay(100); 

  Wire1.beginTransmission(MPU_ADDR1);
  Wire1.write(0x6B);
  Wire1.write(0x80); // Another reset to enter calibration mode
  Wire1.endTransmission(true);
  delay(100);

  Wire1.beginTransmission(MPU_ADDR1);
  Wire1.write(0x6B);
  Wire1.write(0x00); // Wake up MPU6050 and exit calibration mode
  Wire1.endTransmission(true);
  delay(100);

   // Read and discard the first set of readings after calibration
  Wire1.beginTransmission(MPU_ADDR1);
  Wire1.write(0x3B); // Start of accelerometer data registers
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_ADDR1, 6, true);
  while (Wire.available()) {
    Wire1.read();
  }

  Serial.println("First MPU6050 calibration complete.");
}
void calibration2() {
  Wire2.beginTransmission(MPU_ADDR2);
  Wire2.write(0x6B);
  Wire2.write(0x80); // reset the MPU6050
  Wire2.endTransmission(true);
  delay(100); 

  Wire2.beginTransmission(MPU_ADDR2);
  Wire2.write(0x6B);
  Wire2.write(0x00); // Wake up from reset
  Wire2.endTransmission(true);
  delay(100); 

  Wire2.beginTransmission(MPU_ADDR2);
  Wire2.write(0x6B);
  Wire2.write(0x80); // Another reset to enter calibration mode
  Wire2.endTransmission(true);
  delay(100);

  Wire2.beginTransmission(MPU_ADDR2);
  Wire2.write(0x6B);
  Wire2.write(0x00); // Wake up MPU6050 and exit calibration mode
  Wire2.endTransmission(true);
  delay(100);

   // Read and discard the first set of readings after calibration
  Wire2.beginTransmission(MPU_ADDR2);
  Wire2.write(0x3B); // Start of accelerometer data registers
  Wire2.endTransmission(false);
  Wire2.requestFrom(MPU_ADDR2, 6, true);
  while (Wire.available()) {
    Wire2.read();
  }

  Serial.println("2nd MPU6050 calibration complete.");
}


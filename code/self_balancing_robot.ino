/* SELF BALANCING ROBOT
 *  Authors: Kensei Suzuki & Yensabro Kanashiro
 *  Date: 12/9/2020
 *  Sensors: MinIMU-9 (magnetometer, accelerometer, gyroscope)
 *  Acutators: 12V DC gearmotor (1:34 gear ratio)
 *  Controller: PID
 *  Credit to: Brett Beauregard for PID library
/***************************************************************************************************/

//LIBRARIES
#include <LSM6.h> //accelerometer & gyroscope
#include <Wire.h> //I2C communication
#include <math.h> //math functions
#include <PID_v1.h> //PID library; input is the tilt angle, output is PWM for the motor speed
#include <LMotorController.h> //uses the L293D H-bridge to control the motors

//GLOBAL VARS AND CONSTS
//for PID Controller
double Kp = 20; //proportional gain
double Kd = 0; //derivative gain
double Ki = 2; //integral gain
double referencePoint = 0; //reference value from sensor (angle)
double setpoint = referencePoint;
double input, output; //input=tilt angle; output=PWM for motor speed

//for H-bridge motor control
const int enA = 10; //Enable 1&2 with analog PWM signal
const int in1 = 4; //Input 1
const int in2 = 5; //Input 2
const int enB = 11; //Enable 3&4 with analog PWM signal
const int in3 = 6; //Input 3
const int in4 = 7; //Input 4
const int minAbsSpeed = 30; //minimum PWM for motor speed
//double motorSpeedFactorLeft = 0.75; //decrease left motor to match right motor
//for monitor output
char report[80];

//for calculations
int16_t accY, accZ; //acclerometer raw measurements
int16_t gyroX, gyroRate, gXZero=279; //gyroscope raw measurement, rate of roll (dps), and offset value
unsigned long currentTime, prevTime = 0, loopTime; //time variables in milliseconds
float accelAngle, gyroAngle=0, currentAngle; //angles from sensors and filtered angle

//INSTANCES
LSM6 imu; //for IMU sensor
PID myPID(&input, &output, &setpoint, Kp, Kd, Ki, DIRECT); //for PID controller
LMotorController motorController(enA, in1, in2, enB, in3, in4, 1, 1);

/***************************************************************************************************/

void setup() {
  Serial.begin(9600); //communication for serial monitor/plotter
  Wire.begin(); //I2C serial communication

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  //Set motor speed (PWM: 0-255). Set to 0 at start.
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  //Check if the imu has been initialized
  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  
  imu.enableDefault(); //enable default settings for the IMU

  myPID.SetMode(AUTOMATIC); //turn the PID on
  
  Serial.println("Initiate balancing.");
}

/***************************************************************************************************/

void loop() {
  getTiltAngle();
  input = currentAngle; //input for the PID is the filtered angle
  //Serial.println(currentAngle);
  
  if (input > 0){
    myPID.SetControllerDirection(REVERSE);
    myPID.Compute();
    output *= -1;
    motorController.move(output, minAbsSpeed); //using the output from the PID, set motor speed
  }
  else {
    myPID.SetControllerDirection(DIRECT);    
    myPID.Compute(); //use the input for the PID controller and get motor speed
    motorController.move(output, minAbsSpeed); //using the output from the PID, set motor speed
  }
  
  delay(100);
}

/***************************************************************************************************/

//PERSONAL FUNCTIONS
void checkIMU(){
  /*Purpose: Outputs the raw measurements from the IMU*/
  imu.read(); //get raw acclerometer and gyroscope data
  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z,
    imu.g.x, imu.g.y, imu.g.z);
  Serial.println(report);

  delay(100); 
}

void getTiltAngle(){
  /*Purpose: Calculates the angle of tilt of the robot using gyroscope and accelerometer*/
  currentTime = millis(); //get current time
  loopTime = currentTime - prevTime; //get elapsed time
  prevTime  = currentTime; //remember current time   
  imu.read(); //get raw acclerometer and gyroscope data
  
  accY = imu.a.y; //get raw accelerometer data in y-direction
  accZ = imu.a.z; //get raw accelerometer data in z-direction
  accelAngle = (atan2(accY, accZ)*RAD_TO_DEG); //calculate the inclination using accelerometer
  if (accelAngle < 0) { 
    accelAngle = 180 - accelAngle*-1; //if-else used to modify positions of angles
  }
  else{
    accelAngle = -1*(180 - accelAngle);
  }
  
  gyroX = imu.g.x; //get raw gyro data about x-axis
  gyroRate = (((double) gyroX - gXZero) * 8.75/1000); //calc the angular velocity in dps
  gyroAngle = gyroAngle + gyroRate * ((double) loopTime/1000); //calc gyro angle in degrees
  currentAngle = 0.80 * (gyroAngle) + 0.20*(accelAngle); //use a comparative filter for angles
 
}

void getTiltAngle2() {
  /*Purpose: Calculates the angle of tilt of the robot using gyroscope and accelerometer*/
  imu.read(); //get raw acclerometer and gyroscope data
  
  accY = imu.a.y; //get raw accelerometer data in y-direction
  accZ = imu.a.z; //get raw accelerometer data in z-direction
  accelAngle = ((atan2(accY, accZ)+M_PI)*RAD_TO_DEG); //calculate the inclination using accelerometer
  
  gyroX = imu.g.x; //get raw gyro data about x-axis
  gyroRate = (((double)gyroX - 279)*(8.75/1000));
  gyroAngle = gyroAngle +gyroRate*((double)loopTime/1000);
  currentAngle = ((double)0.98 * ( gyroAngle)) + ((double)0.02*(accelAngle));
  
  currentAngle = 0.98 * (currentAngle + gyroAngle) + 0.02*(accelAngle); //use a comparative filter for angles
  //currentAngle *= 999/1000;
  delay(200); 

  //Serial.println(currentAngle);
}

void calibrateGyro() { 
  /*Purpose: Find the offset for imu.g.x value*/
  delay(1000); //wait one second
  int16_t total = 0; //initialize total count to zero
  
  for (int i = 0; i < 100; i++)
  {
    imu.read(); //get raw data
    total += imu.g.x; //sum the values of gyroscope x
    delay(1); //wait 1 millisecond
  }

  gXZero = total / 100;
  Serial.print("gXZero: ");
  Serial.println(gXZero);  
}

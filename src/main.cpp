#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "BluetoothSerial.h"
#include "kalmanfilter.h"
#include <ArduinoEigenDense.h>
#include "easyStepper.h"
#include "MPU6050.h"



// definitions for the stepper motors
#define STEPPER1_DIR_PIN 17
#define STEPPER1_STEP_PIN 16
#define STEPPER2_DIR_PIN 19
#define STEPPER2_STEP_PIN 23

#define STEPPER1_ENABLE_PIN 4 // Enable pin for the stepper motors, not used in this code but can be used to enable/disable the motors
#define STEPPER2_ENABLE_PIN 18
// definitions for the MPU6050
#define MPU6050_SDA 21
#define MPU6050_SCL 22



// time
int current_time = 0;
int previous_time = 0;
int loop_time = 0;

// angle commanded_velocity variables
float current_angle = 0;
float desired_angle = 0;
float stable_angle = 0;
float error = 0;


bool fell = false;



int motor1_speed = 1000000;
int motor2_speed = 1000000;
int previous_speed = 1000000;

float speedinms = 0;
float position = 0;



// commanded_velocity output variables
float commanded_velocity_before_turning = 0;


struct BluetoothData{
  float move = 100;
  float turn = 100;
} bluetoothData; // struct to hold the bluetooth data


// function handles
void IRAM_ATTR drivemotor();
void calibrationsequence();
void updateIMU();
void getBluetoothData(BluetoothData &data);


BluetoothSerial SerialBT;

// create the MPU6050
MPU6050 mpu;     // SDA=21, SCL=22

PIDController anglePID(200.0, 1300.0, 20, 5000.0);
PIDController velocityPID(0.5, 0.5, 0, 5);
lowPassFilter angleLowpass(0.98);
lowPassFilter motorlowpass(0.7);
lowPassFilter motorlowpass2(0.7);


// Constants (also global)
int TIMER_INTERVAL = 10; // in microseconds

constexpr float SAMPLE_TIME = 0.003f;    // Ts
constexpr float GRAVITY     = 9.81f;     // g
constexpr float PEND_L      = 0.05f;      // l
constexpr float MASS_CART   = 0.95;      // M
constexpr float MASS_PEND   = 0.3f;      // m
constexpr float RADIUS = 0.045f; // radius of the wheel

constexpr int MICROSTEPPING = 4;
constexpr float STEPS_PER_REV = 200.0f * MICROSTEPPING; // steps per revolution
constexpr float DISTANCE_PER_STEP = (3.14 * 2 * RADIUS)/STEPS_PER_REV ; // distance per step

constexpr float DITANSCE_PER_ROTATION = 2 * 3.14 * RADIUS; // distance per rotation

constexpr float MICROSECONDS_PER_STEP_AT_1_METER = 1/((1.0f / DITANSCE_PER_ROTATION)*STEPS_PER_REV/1e6); // rotations per second




KalmanFilter<4,4,1> kalman; // 4 states, 3 measurements, 1 input

// Initial state & covariance (globals)
Eigen::Matrix<float,4,1> x0;
Eigen::Matrix<float,4,4> P0;

// Call this once from setup()

void kalmansetup() { 
  // 1) Discrete‐time A matrix (forward‐Euler of your linearized model)
  kalman.A << 
    0,                1,                        0,                                            0,                               
    0,                0,                        -GRAVITY,                               0,                
    0,                0,                         0,                                          1,                 
    0,                0,                -GRAVITY/PEND_L,                               0;     

  // 2) Discrete‐time B matrix (modelled commanded_acceleration → state)
  //    Here u is the horizontal cart commanded_acceleration
  kalman.B << 
        0,
        1,
        0,
        -1/PEND_L;


  // 3) Measurement model H: [ accel‐based θ ; gyro‐based θ̇ ]
  kalman.H << 
    1, 0, 0, 0, 
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

  // 4) Process noise covariance Q (tune these!)
  //    Base it on how wrong you think A/B are or how much unmodelled
  //    acceleration you expect.
  kalman.Q << 
    1,    0,      0,      0,
    0,   1,      0,      0,
    0,   0,      0.3,     0,
    0,    0,     0,    0.1;

  // 5) Measurement noisse covariance R (based on your IMU specs)
  //    First row = variance of θ_acc (converted from m/s² noise via small‐angle)
  //    Second row = variance of gyro-rate
  kalman.R << 
    0.001,  0,    0,       0,
    0,    0.01,    0,        0,
    0,    0,     0.03,    0,
    0,    0,      0,     0.06;



  // 6) Initial filter state P0 and x0
  x0.setZero();                     // start at zero state
  P0 = Eigen::Matrix<float,4,4>::Identity() * 1.0f;  

  kalman.discretize_state_matricies(SAMPLE_TIME); // Discretize the state matrices
  kalman.init(x0, P0);
}


LQR<4, 1> LQRController; // 4 states, 1 input
void LQRsetup() {


  //non discretized 
  // LQRController.A << 
  //   0,                1,                        0,                                            0,                               
  //   0,                0,       (MASS_PEND*GRAVITY/MASS_CART),                               0,                
  //   0,                0,                         0,                                          1,                 
  //   0,                0,        -(MASS_CART+MASS_PEND)*GRAVITY/(MASS_CART*PEND_L),             0;     
  
  LQRController.A << 
    0,                1,                        0,                                            0,                               
    0,                0,                        -GRAVITY,                               0,                
    0,                0,                         0,                                          1,                 
    0,                0,                -GRAVITY/PEND_L,                               0;     
  // 2) Discrete‐time B matrix (modelled commanded_acceleration → state)
  //    Here u is the horizontal cart commanded_acceleration
  // LQRController.B << 
  //   0,
  //   1 / (MASS_CART * RADIUS),
  //   0,
  //  -1 / (MASS_CART * PEND_L * RADIUS);


    LQRController.B << 
        0,
        1,
        0,
        -1/PEND_L;


  LQRController.Q << 
    0.08,    0,        0,      0,    
    0,      0.08,      0,      0,    
    0,        0,      100,      0,    
    0,         0,       0,      0.01;


  LQRController.R << 
    0.03; // Control cost weighting matrix

  LQRController.discretize_state_matricies(SAMPLE_TIME);

  LQRController.init();




}



StepperMotor motor1(STEPPER1_DIR_PIN, STEPPER1_STEP_PIN); // create the stepper motors
StepperMotor motor2(STEPPER2_DIR_PIN, STEPPER2_STEP_PIN); // create the stepper motors

void setup()
{

  anglePID.set_derivative_smoothing(0.995);

  Serial.begin(115200);  

  StepperMotor::setMotors(&motor1, &motor2); // set the motors for the stepper motor class

  StepperMotor::initTimer(20); // initialize the timer for the stepper motors
  // timer1 = timerBegin(0, 80, true); // ISR timer for the stepper motors
  // timerAlarmEnable(timer1);
  // timerAttachInterrupt(timer1, &drivemotor, true);


  SerialBT.begin("stadig"); // Bluetooth device name


  
  mpu.init(MPU6050_SDA, MPU6050_SCL); // initialize the MPU6050

  Wire.write(0);

  calibrationsequence();  

  kalmansetup();

  LQRsetup();

  pinMode(STEPPER1_ENABLE_PIN, OUTPUT); // set the enable pin for the stepper motors
  digitalWrite(STEPPER1_ENABLE_PIN, LOW); // enable the stepper motors

  pinMode(STEPPER2_ENABLE_PIN, OUTPUT); // set the enable pin for the stepper motors
  digitalWrite(STEPPER2_ENABLE_PIN, LOW); // enable the stepper motors


  Serial.println("Setup complete"); // print a message to the serial monitor


}






void loop()
{



  // time management
  current_time = micros();
  loop_time = current_time - previous_time;
  previous_time = current_time;

  
  float travelled_distance = DISTANCE_PER_STEP * motor1.stepCounter_; // distance in meters
  speedinms = travelled_distance/(loop_time/1e6f);

  motor1.stepCounter_ = 0; // reset the step counter for the next loop
  if(commanded_velocity_before_turning < 0){
    speedinms = - speedinms;
  }

  getBluetoothData(bluetoothData); // get the bluetooth data
  float recieved_velocity = map(bluetoothData.move, 0, 200, -100, 100)*0.01; // map the bluetooth data to a suitable range for the motors
  speedinms += recieved_velocity;  // update the position based on the bluetooth data

  position += speedinms * SAMPLE_TIME; // update the position


  mpu.readSensorData(); // read the sensor data
 
  //simple complementary filter
  current_angle = 0.98 * (current_angle + mpu.data.gyroRateY_dt) + 0.02 * mpu.data.accAngleY;

  Eigen::Matrix<float,1,1> u;     u << 0;
  Eigen::Matrix<float,4,1> z;     z << position, speedinms, current_angle*DEG_TO_RAD-stable_angle*DEG_TO_RAD, mpu.data.gyroRateY*DEG_TO_RAD; // Convert to degrees for printing
  
  kalman.predict(u);
  kalman.update(z);
  

  // Get state estimate from Kalman Filter and pass it to the LQR controller
  Eigen::Matrix<float, 4, 1> x_est = kalman.state();
  Eigen::Matrix<float,4,1> x_lqr;
  //x_lqr << -x_est;



  //x_lqr << -position, -speedinms, -current_angle*DEG_TO_RAD, -mpu.data.gyroRateY*DEG_TO_RAD; // Convert to radians for LQR controller 
  x_lqr << -x_est;
  
  auto u2 = LQRController.computeControl(x_lqr); 


  // get commanded acceleration from the LQR controller and turn it to a velocity

  commanded_velocity_before_turning += u2(0)*SAMPLE_TIME; // update the commanded acceleration based on the bluetooth data

  //commanded_velocity_before_turning = motorlowpass.update(commanded_velocity_before_turning); // low pass filter the commanded acceleration
  float recieved_turn = map(bluetoothData.turn, 0, 200, -100, 100)*0.0023; // map the turn value to a suitable range for the motors
  //now limit based on the commanded_velocity_before_turning
  recieved_turn = constrain(recieved_turn, -100+0.5*commanded_velocity_before_turning, 100-0.5*commanded_velocity_before_turning); // limit the turn value to the commanded velocity

  float motor1_velocity  = commanded_velocity_before_turning - recieved_turn;
  float motor2_velocity = commanded_velocity_before_turning + recieved_turn;

  float commanded_velocity_motor1 = MICROSECONDS_PER_STEP_AT_1_METER/motor1_velocity; // Scale the commanded_acceleration to a suitable range for the motors
  float commanded_velocity_motor2 = MICROSECONDS_PER_STEP_AT_1_METER/motor2_velocity; // Scale the commanded_acceleration to a suitable range for the motors

  


  

  

  // motor1_speed = 1000000*(1/((abs(PID_motor1))+1));
  // motor2_speed = 1000000*(1/((abs(PID_motor2))+1));



  // motor1_speed = abs(commanded_velocity);
  // motor2_speed = abs(commanded_velocity);

  
  // if(PID_motor1 < 0){
  //   motor1_speed = -motor1_speed;
  // }
  // if(PID_motor2 < 0){
  //   motor2_speed = -motor2_speed;
  // }



  // MOTOR CONTROL ######################################################




  if(abs(current_angle) > 35){ // fall detection
    fell = true;
    digitalWrite(STEPPER1_ENABLE_PIN, HIGH); // enable the stepper motors
    digitalWrite(STEPPER2_ENABLE_PIN, HIGH);
    StepperMotor::stopTimer(); // stop the stepper motors

  }

  else if(abs(current_angle) < 1 && fell == true){ // fall detection reset
    fell = false;
    delay(1500); // wait for a second to stabilize
    position = 0; // reset the position
    motor1.stepCounter_ = 0;
    current_angle = 0; // reset the angle
    speedinms = 0; // reset the speed
    commanded_velocity_before_turning = 0; // reset the commanded velocity
    Serial.println("Resetting position and angle after fall"); // print a message to the serial monitor
    digitalWrite(STEPPER1_ENABLE_PIN, LOW); // disable the stepper motors
    digitalWrite(STEPPER2_ENABLE_PIN, LOW);
    StepperMotor::startTimer(); // start the stepper motors timer
    
    
  }  




  motor1.setSpeed(-commanded_velocity_motor1); // set the speed of the motors
  motor2.setSpeed(commanded_velocity_motor2); // set the speed of the motors

  Serial.println(commanded_velocity_motor1);
  Serial.println(commanded_velocity_motor2);



}



// void IRAM_ATTR drivemotor() {

//   if (step_motor1_counter >= motor1_speed_multiple) {
//     digitalWrite(STEPPER1_STEP_PIN, HIGH);
//     digitalWrite(STEPPER1_STEP_PIN, LOW);
//     step_motor1_counter = 0;
//     step_counter++;
//   }
  
//   if (step_motor2_counter >= motor2_speed_multiple) {
//     digitalWrite(STEPPER2_STEP_PIN, HIGH);
//     digitalWrite(STEPPER2_STEP_PIN, LOW);
//     step_motor2_counter = 0;
//   }

//   step_motor1_counter++;
//   step_motor2_counter++;

// }


void calibrationsequence(){
  
  mpu.readSensorData(); // read the sensor data again
  stable_angle = mpu.data.accAngleY;
  delay(2000); // wait for a short time to stabilize
  mpu.readSensorData(); // read the sensor data again
  stable_angle = (stable_angle + mpu.data.accAngleY) / 2.0f; // average the two readings
  desired_angle = stable_angle; // set the desired angle to the stable angle
  Serial.print("Stable angle: ");
  Serial.println(stable_angle); // print the stable angle in degrees
}




void getBluetoothData(BluetoothData &data) {
  if (SerialBT.available()){
    data.move = (float)SerialBT.read(); // read the move command
    data.turn = (float)SerialBT.read(); // read the turn command

    data.move = constrain(data.move, 0, 200); // constrain the move command to a range of 0 to 200
    data.turn = constrain(data.turn, 0, 200); // constrain the turn command to a range of 0 to 200
  }
}


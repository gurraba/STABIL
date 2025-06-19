// easyStepper.cpp
#include "easyStepper.h"

// Define static members


StepperMotor* StepperMotor::motor1_ = nullptr;
StepperMotor* StepperMotor::motor2_ = nullptr;
hw_timer_t* StepperMotor::timer_ = NULL;
int StepperMotor::timerInterval_ = 50;

// Constructor
StepperMotor::StepperMotor(int dirPin, int stepPin) {
    dirPin_ = dirPin;           // Direction pin
    stepPin_ = stepPin;         // Step pin
    speedMultiple_ = 100000;         // Default speed multiple, declares how many times should the timer fire before stepping
    stepSkipCounter_ = 0;           // Step counter
    direction_ = 1;             // Default direction (HIGH)
  
    pinMode(dirPin_, OUTPUT);
    pinMode(stepPin_, OUTPUT);
}

// Initialize the shared timer

void StepperMotor::initTimer(int timerInterval, int prescaler) {
    
    timerInterval_ = timerInterval; // Store the interval
    timer_ = timerBegin(0, prescaler, true); // 80 prescaler for 1MHz
    timerAttachInterrupt(timer_, StepperMotor::onTimer, true);
    timerAlarmWrite(timer_, timerInterval_, true);
    timerAlarmEnable(timer_);
    
}

void StepperMotor::stopTimer() {
    if (StepperMotor::timer_) {
        timerAlarmDisable(StepperMotor::timer_);
    }
}

void StepperMotor::startTimer() {
    if (StepperMotor::timer_) {
        timerAlarmEnable(StepperMotor::timer_);
    }
}

// Register both motors
void StepperMotor::setMotors(StepperMotor* motor1, StepperMotor* motor2) {
    motor1_ = motor1;
    motor2_ = motor2;
}

// Set motor speed and direction
void StepperMotor::setSpeed(int speed) {
    speedMultiple_ = abs(speed) / timerInterval_; // Adjust speed by interval
    direction_ = (speed >= 0) ? HIGH : LOW; // Positive speed = forward
    digitalWrite(dirPin_, direction_);

    //we cannot step faster than once every 150 microseconds. This will cause the motor to skip steps in many cases.a

    if (speedMultiple_ < 150/timerInterval_) {
        speedMultiple_ = 150/timerInterval_; // Minimum speed multiple
    }


}

// Step the motor (not used directly, kept for clarity)

// ISR to step both motors
void StepperMotor::onTimer() {
    if (motor1_) {
        if (motor1_->stepSkipCounter_ >= motor1_->speedMultiple_) {
            digitalWrite(motor1_->stepPin_, HIGH);
            digitalWrite(motor1_->stepPin_, LOW);
            motor1_->stepSkipCounter_ = 0;
            motor1_->stepCounter_++; // Increment step counter for motor1
        }
        motor1_->stepSkipCounter_++;
        
    }
    if (motor2_) {
        if (motor2_->stepSkipCounter_ >= motor2_->speedMultiple_) {
            digitalWrite(motor2_->stepPin_, HIGH);
            digitalWrite(motor2_->stepPin_, LOW);
            motor2_->stepSkipCounter_ = 0;
            motor2_->stepCounter_++; // Increment step counter for motor1
        }
        motor2_->stepSkipCounter_++;
    }
}
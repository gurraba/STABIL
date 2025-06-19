// easyStepper.h
#ifndef EASY_STEPPER_H
#define EASY_STEPPER_H

#include <Arduino.h>

class StepperMotor {
public:

    static volatile bool timerFired_;
    // Constructor to initialize pins
    StepperMotor(int dirPin, int stepPin);

    // Initialize the shared timer with user-specified interval (call once in setup)
    static void initTimer(int timerInterval, int prescaler = 80);

    // Register both motors (call once in setup)
    static void setMotors(StepperMotor* motor1, StepperMotor* motor2);

    // Start and stop the timer
    static void startTimer();
    static void stopTimer();
    

    // Set motor speed and direction
    void setSpeed(int speed);

    int dirPin_, stepPin_;        // Motor pins
    int speedMultiple_;           // Steps to skip before stepping
    int stepSkipCounter_;             // Current step counter
    int direction_;               // HIGH for forward, LOW for reverse
    int stepCounter_ = 0;          // Total steps taken (not used in ISR)

    // Static ISR to step both motors
    static void IRAM_ATTR onTimer();

    // Static members for ISR and timer
    static StepperMotor* motor1_;     // First motor
    static StepperMotor* motor2_;     // Second motor
    static  hw_timer_t* timer_;        // Shared timer
    static int timerInterval_;        // Shared timer interval
};

#endif // EASY_STEPPER_H
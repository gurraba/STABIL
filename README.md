# STABIL
## A remote controlled self balancing robot 

The code and android application avaiable in this repo is used to control a two wheeled self balancing robot. '
Self balancing robots are a somewhat common first project, but they can be quite challenging to get working correctly.
This repo may be of help if you need inspiration on how to make a similar robot. 

## What is provided in this repo?
This repo contains several DIY "libarries" that may be of use if you are considering a similar project. These files have been
made during this project, for the specific purpose of this self balancing robot, so the functions provided should be very much relevant.
### MPU6050.h allows for easy retrieval of data from an mpu6050 accelerometer. 
### easyStepper.h/cpp is made to give the ability to change speed of stepper motors as fast as possible.
### kalmanfilter.h contains not code for implementing a kalman filter, but also low pass filters, PID controllers and LOR controllers. 
  
###Specific challanges
This robot utilizes stepper motors. This adds quite a bit of complexity and annoyance due to the nature of how stepper motors are controlled.



#include <ArduinoEigenDense.h>

#pragma once
template<int n, int m, int p>
struct KalmanFilter {

  using Vecx = Eigen::Matrix<float,n,1>;
  using Vecz = Eigen::Matrix<float,m,1>;
  using MatA = Eigen::Matrix<float,n,n>;
  using MatB = Eigen::Matrix<float,n,p>;
  using MatH = Eigen::Matrix<float,m,n>;
  using MatP = Eigen::Matrix<float,n,n>;
  using MatQ = Eigen::Matrix<float,n,n>;
  using MatR = Eigen::Matrix<float,m,m>;
  using MatPsi = Eigen::Matrix<float, n, n>;  // Psi matrix
  using MatI = Eigen::Matrix<float, n, n>;  // Identity matrix
  using MatK = Eigen::Matrix<float,n,m>;
  MatA A;      // state transition
  MatB B;      // control input
  MatH H;      // measurement model
  MatQ Q;      // process noise cov
  MatR R;      // measurement noise cov
  MatP P;      // posterior covariance
  Vecx x;      // posterior state
  MatI I = MatI::Identity();  // Identity matrix
  MatK K;      // Kalman gain

  KalmanFilter() = default;

  void init(const Vecx& x0, const MatP& P0) {
    x = x0;
    P = P0;
  }


  void discretize_state_matricies(float Ts) {
            // Discretization (Taylor series)

      MatPsi Psi = MatPsi::Zero();
      Psi = I * Ts + (A*(pow(Ts,2) / 2)) + (A*A*(pow(Ts,3) / 6)) + (A*A*A*(pow(Ts,4) / 24)) + (A*A*A*A*(pow(Ts,5) / 120));
      A = I + A * Psi;
      B = Psi * B;
    }


  void predict(const Eigen::Matrix<float,p,1>& u) {
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
  }

// Update with measurement z
void update(const Vecz& z) {
    // 1) Compute residual covariance as a concrete matrix
    MatR S = H * P * H.transpose() + R;

    // 2) Invert into a concrete MatR
    MatR  S_inv = S.inverse();

    // 3) Check for NaNs and fallback
    if (S_inv.hasNaN()) {
        Serial.println("S_inv has NaN, using identity");
        MatR S_inv = MatR::Identity();
    }

    // 4) Kalman gain
    K = P * H.transpose() * S_inv;

    // 5) Innovation
    Vecz y = z - H * x;

    // 6) State update
    x += K * y;

    // 7) Covariance update
    P = (MatP::Identity() - K * H) * P;
}

const Vecx& state() const { return x; }

};






class lowPassFilter {
public:
// alpha is the weight of the previous value, higher is smoother at rhe cost of responsiveness
  lowPassFilter(float alpha){
    alpha_ = alpha;
    prev_value_ = 0; // Initialize previous value
  }

  float update(float value) {
    float filtered_value = (1 - alpha_) * value + alpha_ * prev_value_;
    prev_value_ = filtered_value;
    return filtered_value;
    }

private:
  float alpha_; // Weight of the previous value
  float prev_value_; // Previous value

};







// CONTROLLER CLASSES -------------------------------------------------------------------------------------------------------------------------------
// CONTROLLER CLASSES -------------------------------------------------------------------------------------------------------------------------------
// CONTROLLER CLASSES -------------------------------------------------------------------------------------------------------------------------------
// CONTROLLER CLASSES -------------------------------------------------------------------------------------------------------------------------------



class PIDController {
public:

  PIDController(float kp, float ki, float kd, float windup){
    kp_ = kp; // Proportional gain
    ki_ = ki; // Integral gain
    kd_ = kd; // Derivative gain
    windup_ = windup; // Windup limit
    d_lowpass_alpha_ = 0; // Initialize derivative smoothing
    dPrev_ = 0; // Initialize previous derivative
  }

  void set_derivative_smoothing(float derivative_smoothing) {
    d_lowpass_alpha_ = derivative_smoothing; // alpha is the weight of the previous value
  }

  void reset_PID_values() {
    p_ = 0; // Reset proportional term
    i_ = 0; // Reset integral term
    d_ = 0; // Reset derivative term
    prevError_ = 0; // Reset previous error
  }

  float get_Integral_value() {
    return(i_); // Get the current integral value
  }
  void set_Integral_value(float integral_value) {
    i_ = integral_value; // Set the current integral value
  }


  // Method to update the PID output
  float update(float error, float dt) {
    p_ = error * kp_;                    // Proportional term
    i_ += error * dt * ki_;              // Integral term
    i_ = constrain(i_, -windup_, windup_);     // Prevent integral windup
    d_ = ((error - prevError_) / dt) * kd_; // Derivative term
    d_ = (1 - d_lowpass_alpha_) * d_ + d_lowpass_alpha_ * dPrev_; // Smooth derivative term
    dPrev_ = d_;

    prevError_ = error;                  // Store error for next cycle
    return p_ + i_ + d_;                 // Return total output
  }

private:
  float kp_, ki_, kd_, windup_, d_lowpass_alpha_, dPrev_;    // PID constants
  float p_ = 0, i_ = 0, d_ = 0; // PID terms
  float prevError_ = 0;   // Previous error for derivative
};





#pragma once

template<int n, int p>
struct LQR {
    // Type definitions using Eigen
    using Vecx = Eigen::Matrix<float, n, 1>;  // State vector
    using Vecu = Eigen::Matrix<float, p, 1>;  // Control input vector
    using MatA = Eigen::Matrix<float, n, n>;  // State transition matrix
    using MatB = Eigen::Matrix<float, n, p>;  // Control input matrix
    using MatQ = Eigen::Matrix<float, n, n>;  // State cost weighting matrix
    using MatR = Eigen::Matrix<float, p, p>;  // Control cost weighting matrix
    using MatK = Eigen::Matrix<float, p, n>;  // Gain matrix
    using MatP = Eigen::Matrix<float, n, n>;  // Riccati solution matrix
    using MatI = Eigen::Matrix<float, n, n>;  // Identity matrix
    using MatPsi = Eigen::Matrix<float, n, n>;  // Psi matrix
    // Public members, similar to Kalman Filter
    MatA A;  // State transition
    MatB B;  // Control input
    MatQ Q;  // State cost
    MatR R;  // Control cost
    MatK K;  // Optimal gain

    MatI I = MatI::Identity();  // Identity matrix


    // Default constructor
    LQR() = default;



    void init(){
      //MatP P = MatP::Zero();  // Initial guess for P
      MatP P = MatP::Identity();  // Initial guess for P

      const int maxIterations = 1000000;  // Iterations for convergence

      for (int i = 0; i < maxIterations; ++i) {
          MatP P_prev = P;  // Store previous P

          P = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;

          if ((P - P_prev).norm() < 1e-3) {  // Check for convergence
              break;
          }
      }

      //compute feedback gain
      K = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);
      Serial.println("BEGIN");
      Serial.println(K(0,0));
      Serial.println(K(0,1));
      Serial.println(K(0,2));
      Serial.println(K(0,3));
      Serial.println("DONE");




    }

    void discretize_state_matricies(float Ts) {
            // Discretization (Taylor series)

        MatPsi Psi = MatPsi::Zero();

        Psi = I * Ts + (A*(pow(Ts,2) / 2)) + (A*A*(pow(Ts,3) / 6)) + (A*A*A*(pow(Ts,4) / 24)) + (A*A*A*A*(pow(Ts,5) / 120));
          A = I + A * Psi;
          B = Psi * B;
    }

    Vecu computeControl(const Vecx& x) {

      return -K * x;  // Compute control input

    }

};


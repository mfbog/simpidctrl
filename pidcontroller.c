#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // For sleep function

// PID controller structure
typedef struct {
    double Kp;  // Proportional gain
    double Ki;  // Integral gain
    double Kd;  // Derivative gain
    double prev_error; // Previous error
    double integral;   // Integral of the error
} PIDController;

// Function to initialize PID controller
void initialize_pid(PIDController *pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
}

// PID compute function
double compute_pid(PIDController *pid, double setpoint, double measured_value, double dt) {
    double error = setpoint - measured_value;
    pid->integral += error * dt;
    double derivative = (error - pid->prev_error) / dt;
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output;
}

int main() {
    PIDController pid;
    double setpoint = 100.0; // Target value
    double measured_value = 0.0; // Initial value
    double dt = 1.0; // Time step (1 second)

    // Initialize PID controller with arbitrary gains
    initialize_pid(&pid, 1.0, 0.1, 0.01);

    // Simulate control loop
    for (int i = 0; i < 60; i++) { // Run for 60 iterations (60 seconds)
        double control_signal = compute_pid(&pid, setpoint, measured_value, dt);
        measured_value += control_signal; // Update the measured value (simulated system response)
        
        // Print current status
        printf("Time: %d s, Setpoint: %.2f, Measured Value: %.2f, Control Signal: %.2f\n",
               i, setpoint, measured_value, control_signal);
        
        sleep(1); // Wait for 1 second
    }

    return 0;
}

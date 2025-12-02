#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

// Define to exclude main() from motor_control_full.cpp
#define MOTOR_CONTROL_NO_MAIN

// Include motor control code (excluding main function)
#include "motor_control_full.cpp"

//------------------------
// Simulation Parameters (Adjustable)
//------------------------
struct SimulationParams {
    // Target position (radians) - ADJUST THIS VARIABLE
    double target_position = 1.0;  // Change this to set desired position
    
    // Motor parameters
    double B_eq     = 6e-5;   // equivalent damping (motor side)
    double J_eq     = 7e-7;   // equivalent inertia (motor side)
    double Ke       = 0.036;
    double R        = 2.6;
    double L        = 0.003;
    double Kt       = 0.036;
    double N_gear   = 50.0;   // motor : output
    
    // Simulation timing
    double Ts       = 1e-4;   // Sampling period [s]
    double simTime  = 1.0;    // Simulation length [s] (increased for better visualization)
    
    // Control parameters
    double wf_vel = 100.0;  // Hz for velocity loop
    double wf_pos = 50.0;   // Hz for position loop
    double umax = 12.0;     // Control saturation (V)
    double kr_vel = 1.0;
    double kr_pos = 1.0;
    double speed_scale = 1.0;
};

//------------------------
// Data Logger for Plotting
//------------------------
class DataLogger {
public:
    void log(double time, double target_pos, double actual_pos, double target_vel, double actual_vel, double control) {
        times.push_back(time);
        target_positions.push_back(target_pos);
        actual_positions.push_back(actual_pos);
        target_velocities.push_back(target_vel);
        actual_velocities.push_back(actual_vel);
        controls.push_back(control);
    }
    
    void saveToCSV(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }
        
        // Write header
        file << "time,target_position,actual_position,target_velocity,actual_velocity,control_voltage\n";
        
        // Write data
        for (size_t i = 0; i < times.size(); ++i) {
            file << times[i] << ","
                 << target_positions[i] << ","
                 << actual_positions[i] << ","
                 << target_velocities[i] << ","
                 << actual_velocities[i] << ","
                 << controls[i] << "\n";
        }
        
        file.close();
        std::cout << "Data saved to " << filename << std::endl;
    }
    
    void printSummary() {
        if (times.empty()) return;
        
        std::cout << "\n=== Simulation Summary ===" << std::endl;
        std::cout << "Final time: " << times.back() << " s" << std::endl;
        std::cout << "Target position: " << target_positions.back() << " rad" << std::endl;
        std::cout << "Final position: " << actual_positions.back() << " rad" << std::endl;
        std::cout << "Position error: " << std::abs(target_positions.back() - actual_positions.back()) << " rad" << std::endl;
        std::cout << "Final velocity: " << actual_velocities.back() << " rad/s" << std::endl;
    }
    
private:
    std::vector<double> times;
    std::vector<double> target_positions;
    std::vector<double> actual_positions;
    std::vector<double> target_velocities;
    std::vector<double> actual_velocities;
    std::vector<double> controls;
};

//------------------------
// Position Reference Generator
//------------------------
class PositionReference {
public:
    PositionReference(double target_pos, double Ts) 
        : target_(target_pos), Ts_(Ts), current_pos_(0.0), current_vel_(0.0) {}
    
    void setTarget(double new_target) {
        target_ = new_target;
    }
    
    double getPosition() const {
        return current_pos_;
    }
    
    double getVelocity() const {
        return current_vel_;
    }
    
    void update() {
        // Simple reference generator: move towards target with limited velocity
        double max_vel = 5.0; // rad/s
        double error = target_ - current_pos_;
        double vel_command = std::clamp(error * 2.0, -max_vel, max_vel); // Simple P controller for reference
        
        // Update position and velocity
        current_vel_ = vel_command;
        current_pos_ += current_vel_ * Ts_;
    }
    
private:
    double target_;
    double Ts_;
    double current_pos_;
    double current_vel_;
};

//------------------------
// Main Simulation Function
//------------------------
int main() {
    SimulationParams params;
    
    // ===== ADJUST TARGET POSITION HERE =====
    params.target_position = 2.0;  // Change this value to set target position (radians)
    // =======================================
    
    int N = static_cast<int>(params.simTime / params.Ts);
    
    // Form discrete state space matrices
    stateSpaceRep sys = formDiscreteMatrices(params.Ts, params.N_gear, params.B_eq, 
                                             params.J_eq, params.Ke, params.R, params.L, params.Kt);
    
    // Initialize observer
    LuenbergerObserver observer(sys);
    
    // Initialize RLS estimators
    MechanicalRLS mechRLS;
    ElectricalRLS elecRLS;
    
    // Initialize plant (for simulation)
    MotorPlant plant(sys);
    
    // Initialize position reference generator
    PositionReference posRef(params.target_position, params.Ts);
    
    // Initialize controllers
    VelocityPIController* velController = nullptr;
    OptimalPIDController* posController = nullptr;
    
    // Variables for derivative estimation
    double omega_in_prev = 0.0;
    double i_prev = 0.0;
    double u_prev = 0.0;
    
    // Data logger
    DataLogger logger;
    
    std::cout << "Starting simulation..." << std::endl;
    std::cout << "Target position: " << params.target_position << " rad" << std::endl;
    std::cout << "Simulation time: " << params.simTime << " s" << std::endl;
    std::cout << "Number of steps: " << N << std::endl;
    
    // Main control loop
    for (int k = 0; k < N; ++k) {
        double t = k * params.Ts;
        
        // Get plant outputs (simulated measurements)
        Vector2d y_meas = plant.outputs();
        double theta_out_meas = y_meas(0);
        double omega_in_meas = y_meas(1);
        double i_meas = plant.current();
        
        // Update position reference
        posRef.update();
        double theta_ref = posRef.getPosition();
        double omega_ref = posRef.getVelocity();
        
        // Step 1: Run Luenberger observer 3 times before RLS (every 4th step)
        if (k % 4 == 0 && k > 0) {
            // Run observer 3 times with current measurement
            for (int obs_iter = 0; obs_iter < 3; ++obs_iter) {
                observer.step(y_meas, u_prev);
            }
            
            // Step 2: Update RLS with observer state estimate
            Vector3d x_hat = observer.state();
            double omega_in_hat = x_hat(1);
            double i_hat = x_hat(2);
            
            // Estimate derivatives using finite differences
            double omega_in_dot = (omega_in_hat - omega_in_prev) / params.Ts;
            double i_dot = (i_hat - i_prev) / params.Ts;
            
            // Update mechanical RLS
            double tauL_eq = 0.0;
            mechRLS.update(omega_in_dot, omega_in_hat, i_hat, tauL_eq);
            
            // Update electrical RLS
            elecRLS.update(i_dot, i_hat, omega_in_hat, u_prev);
            
            // Step 3: Solve LQR for optimal gain matrices
            // Solve velocity LQR (PI-like)
            VelocityLQRSolver velLQR(sys, params.wf_vel);
            MatrixXd Kv = velLQR.solveLQR();
            
            // Solve position LQR (PID-like)
            computeOptimalGain posLQR(sys, params.wf_pos, true); // true = position mode
            MatrixXd Kp = posLQR.solveLQR();
            
            // Step 4: Create/update controllers
            if (velController != nullptr) {
                delete velController;
            }
            velController = new VelocityPIController(Kv, params.Ts, params.umax, params.kr_vel);
            
            if (posController != nullptr) {
                delete posController;
            }
            posController = new OptimalPIDController(Kp, params.wf_pos, params.Ts, params.umax, params.kr_pos);
            
            // Update previous values for next RLS update
            omega_in_prev = omega_in_hat;
            i_prev = i_hat;
        } else {
            // Normal step: update observer once
            observer.step(y_meas, u_prev);
        }
        
        // Step 5: Compute control
        Vector3d x_hat = observer.state();
        double u = 0.0;
        
        // Use position controller if available, otherwise velocity controller
        if (posController != nullptr) {
            posController->updateState(x_hat);
            u = posController->computeControl(theta_ref, theta_out_meas);
        } else if (velController != nullptr) {
            velController->updateState(x_hat);
            double omega_ref_scaled = params.speed_scale * omega_ref;
            u = velController->computeControl(omega_ref_scaled, x_hat(1));
        }
        
        // Apply control to plant
        plant.step(u);
        u_prev = u;
        
        // Log data (every 10th step to reduce file size)
        if (k % 10 == 0) {
            logger.log(t, theta_ref, theta_out_meas, omega_ref, omega_in_meas, u);
        }
        
        // Initialize previous values on first iteration
        if (k == 0) {
            omega_in_prev = x_hat(1);
            i_prev = x_hat(2);
        }
    }
    
    // Cleanup
    if (velController != nullptr) {
        delete velController;
    }
    if (posController != nullptr) {
        delete posController;
    }
    
    // Save data and print summary
    logger.saveToCSV("motor_simulation_data.csv");
    logger.printSummary();
    
    std::cout << "\nTo plot the data, run: python plot_motor_data.py" << std::endl;
    std::cout << "Or open motor_simulation_data.csv in Excel/Matlab" << std::endl;
    
    return 0;
}


# Motor Control System

A comprehensive C++ implementation of an advanced motor control system featuring state estimation, parameter identification, and optimal control using LQR (Linear Quadratic Regulator) design.

## Overview

This motor control system implements a complete control architecture for a DC motor with gearbox, including:

- **State Estimation**: Luenberger observer for unmeasured states
- **Parameter Identification**: Recursive Least Squares (RLS) for mechanical and electrical parameters
- **Optimal Control**: LQR-based controllers for both velocity (PI-like) and position (PID-like) control
- **Simulation Tools**: Complete simulation environment with data logging and plotting

## System Architecture

The control system follows this architecture:

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Plant     │────▶│   Observer   │────▶│     RLS     │
│  (Motor)    │     │ (Luenberger) │     │ (Parameter  │
└──────┬──────┘     └──────┬───────┘     │  Estimation)│
       │                   │             └──────┬───────┘
       │                   │                    │
       ▼                   ▼                    ▼
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│  Measured  │     │   State      │     │   System    │
│  Outputs   │     │   Estimate   │     │   Matrices  │
└─────────────┘     └──────┬───────┘     └──────┬───────┘
                           │                    │
                           └──────────┬──────────┘
                                      ▼
                           ┌──────────────────┐
                           │   LQR Solver      │
                           │  (Optimal Gains)  │
                           └─────────┬─────────┘
                                     │
                                     ▼
                           ┌──────────────────┐
                           │   Controller     │
                           │  (PI/PID-like)   │
                           └─────────┬─────────┘
                                     │
                                     ▼
                           ┌──────────────────┐
                           │   Control        │
                           │   Voltage (u)    │
                           └──────────────────┘
```

## Control Algorithm

The control loop executes the following steps:

1. **Observer Update**: Luenberger observer estimates the full state vector from measurements
2. **RLS Update** (periodic): Recursive Least Squares identifies motor parameters
   - Mechanical parameters: `J_eq`, `B_eq`, `Kt`
   - Electrical parameters: `R`, `L`, `Ke`
3. **LQR Solution**: Solves the Discrete Algebraic Riccati Equation (DARE) to compute optimal gains
4. **Control Computation**: Applies optimal control law with saturation

### Control Loop Sequence

Every control step:
- Update observer with current measurements
- Compute control using LQR gains
- Apply control to plant

Every 4th step (periodic):
- Run observer 3 times for better convergence
- Update RLS estimators
- Re-solve LQR for updated system parameters
- Update controller gains

## Components

### Core Classes

#### `stateSpaceRep`
State-space representation structure containing:
- `Ad`: Discrete-time state matrix (3×3)
- `Bd`: Discrete-time input matrix (3×1) - voltage input
- `Ed`: Discrete-time disturbance matrix (3×1) - load torque
- `C`: Output matrix (2×3) - outputs: [theta_out, omega_in]

#### `formDiscreteMatrices()`
Creates and discretizes the continuous-time state-space model using matrix exponential.

**State Vector**: `x = [theta_out; omega_in; i]`
- `theta_out`: Output angle (rad)
- `omega_in`: Motor angular velocity (rad/s)
- `i`: Motor current (A)

**Parameters**:
- `Ts`: Sampling period (s)
- `N_gear`: Gear ratio
- `B_eq`: Equivalent viscous damping
- `J_eq`: Equivalent inertia
- `Ke`: Back-EMF constant
- `R`: Resistance (Ω)
- `L`: Inductance (H)
- `Kt`: Torque constant (N·m/A)

### Estimation Classes

#### `LuenbergerObserver`
State observer for estimating unmeasured states (current).

**Update Law**:
```
x̂_{k+1} = Ad·x̂_k + Bd·u_k + L·(y_k - C·x̂_k)
```

#### `MechanicalRLS`
Recursive Least Squares estimator for mechanical parameters:
- Estimates: `Kt/J_eq`, `B_eq/J_eq`, `1/J_eq`
- Uses: `omega_in`, `omega_in_dot`, `i`, `tau_load_eq`

#### `ElectricalRLS`
Recursive Least Squares estimator for electrical parameters:
- Estimates: `-R/L`, `-Ke/L`, `1/L`
- Uses: `i`, `i_dot`, `omega_in`, `V`

### Control Classes

#### `VelocityLQRSolver`
Solves LQR for velocity control with integral action (PI-like).
- Augmented state: `[theta_out; omega_in; i; e_int]` (4 states)
- Output: Optimal gain matrix `Kv` (1×4)

#### `VelocityPIController`
Velocity controller using LQR-designed PI gains.
- Input: Velocity reference, measured velocity
- Output: Control voltage

#### `computeOptimalGain`
Solves LQR for position control with integral and derivative action (PID-like).
- Augmented state: `[theta_out; omega_in; i; e_int; e_der]` (5 states)
- Output: Optimal gain matrix `Kp` (1×5)
- Mode: `false` for velocity, `true` for position

#### `OptimalPIDController`
Position controller using LQR-designed PID gains.
- Input: Position reference, measured position
- Output: Control voltage
- Features: Derivative filtering, integral action, saturation

### Simulation Classes

#### `MotorPlant`
Simulates the motor plant dynamics.
- Methods: `step(u, tauL_eq)`, `outputs()`, `thetaOut()`, `omegaIn()`, `current()`

## Dependencies

- **Eigen3**: Linear algebra library
  - Core: `Eigen/Dense`
  - Matrix functions: `unsupported/Eigen/MatrixFunctions` (for matrix exponential)
- **C++17** or later
- **Standard Library**: `<iostream>`, `<limits>`, `<fstream>`, `<vector>`, `<cmath>`

### Installing Eigen

**Linux/Mac**:
```bash
sudo apt-get install libeigen3-dev  # Ubuntu/Debian
brew install eigen                   # macOS
```

**Windows**:
Download from [Eigen website](https://eigen.tuxfamily.org/) and add to include path.

## File Structure

```
motor_control/
├── motor_control_full.cpp    # Main control system implementation
├── motor_simulation.cpp      # Simulation with plotting
├── plot_motor_data.py         # Python plotting script
├── README.md                  # This file
└── README_SIMULATION.md       # Simulation-specific documentation
```

## Usage

### Basic Usage

```cpp
#include "motor_control_full.cpp"

// Define motor parameters
double B_eq = 6e-5;
double J_eq = 7e-7;
double Ke = 0.036;
double R = 2.6;
double L = 0.003;
double Kt = 0.036;
double N_gear = 50.0;
double Ts = 1e-4;

// Create system
stateSpaceRep sys = formDiscreteMatrices(Ts, N_gear, B_eq, J_eq, Ke, R, L, Kt);

// Initialize observer
LuenbergerObserver observer(sys);

// Initialize RLS
MechanicalRLS mechRLS;
ElectricalRLS elecRLS;

// Control loop
for (int k = 0; k < N; ++k) {
    // Get measurements
    Vector2d y_meas = getMeasurements();
    
    // Update observer
    observer.step(y_meas, u_prev);
    
    // Periodic RLS update
    if (k % 4 == 0 && k > 0) {
        // Run observer 3 times
        for (int i = 0; i < 3; ++i) {
            observer.step(y_meas, u_prev);
        }
        
        // Update RLS and solve LQR
        // ... (see motor_control_full.cpp for details)
    }
    
    // Compute control
    Vector3d x_hat = observer.state();
    double u = computeControl(x_hat, reference);
    
    // Apply control
    applyControl(u);
}
```

### Running the Simulation

1. **Compile**:
   ```bash
   g++ -std=c++17 -I/path/to/eigen motor_simulation.cpp -o motor_simulation
   ```

2. **Adjust target position** in `motor_simulation.cpp`:
   ```cpp
   params.target_position = 2.0;  // radians
   ```

3. **Run**:
   ```bash
   ./motor_simulation
   ```

4. **Plot results**:
   ```bash
   python plot_motor_data.py
   ```

## Parameters

### Motor Parameters (Default Values)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `B_eq` | 6e-5 | Equivalent viscous damping (N·m·s/rad) |
| `J_eq` | 7e-7 | Equivalent inertia (kg·m²) |
| `Ke` | 0.036 | Back-EMF constant (V·s/rad) |
| `R` | 2.6 | Resistance (Ω) |
| `L` | 0.003 | Inductance (H) |
| `Kt` | 0.036 | Torque constant (N·m/A) |
| `N_gear` | 50.0 | Gear ratio (motor:output) |

### Control Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `Ts` | 1e-4 | Sampling period (s) |
| `wf_vel` | 100.0 | Velocity loop filter frequency (Hz) |
| `wf_pos` | 50.0 | Position loop filter frequency (Hz) |
| `umax` | 12.0 | Control saturation limit (V) |
| `kr_vel` | 1.0 | Velocity reference prefilter gain |
| `kr_pos` | 1.0 | Position reference prefilter gain |
| `speed_scale` | 1.0 | Speed scaling factor |

### Tuning Guidelines

1. **Observer Gains**: Adjust `L_` matrix in `LuenbergerObserver` constructor
2. **RLS Initialization**: Modify `P_` initial covariance in RLS constructors
3. **LQR Weights**: Adjust `Q` and `R` matrices in LQR solvers
4. **Filter Frequencies**: Tune `wf_vel` and `wf_pos` for desired response
5. **Saturation**: Set `umax` based on motor voltage limits

## Control Modes

### Velocity Control
- Uses `VelocityPIController` with LQR-designed PI gains
- Suitable for speed regulation
- Input: Velocity reference (rad/s)

### Position Control
- Uses `OptimalPIDController` with LQR-designed PID gains
- Suitable for position tracking
- Input: Position reference (rad)
- Features derivative filtering for noise reduction

## Mathematical Background

### State-Space Model

**Continuous-time**:
```
ẋ = A·x + B·u + E·τ_load
y = C·x
```

**Discrete-time** (via matrix exponential):
```
x_{k+1} = Ad·x_k + Bd·u_k + Ed·τ_load_k
y_k = C·x_k
```

### LQR Problem

Minimize cost function:
```
J = Σ [x'·Q·x + u'·R·u]
```

Solution via DARE:
```
P = Q + A'·P·A - A'·P·B·(R + B'·P·B)⁻¹·B'·P·A
K = (R + B'·P·B)⁻¹·B'·P·A
```

### RLS Algorithm

Parameter update:
```
K(k) = P(k-1)·φ(k) / (1 + φ'(k)·P(k-1)·φ(k))
θ(k) = θ(k-1) + K(k)·(y(k) - φ'(k)·θ(k-1))
P(k) = P(k-1) - K(k)·φ'(k)·P(k-1)
```

## Examples

### Example 1: Velocity Control

```cpp
// Set velocity reference
double omega_ref = 10.0; // rad/s

// Solve velocity LQR
VelocityLQRSolver velLQR(sys, wf_vel);
MatrixXd Kv = velLQR.solveLQR();

// Create controller
VelocityPIController controller(Kv, Ts, umax, kr_vel);

// Control loop
Vector3d x_hat = observer.state();
controller.updateState(x_hat);
double u = controller.computeControl(omega_ref, x_hat(1));
```

### Example 2: Position Control

```cpp
// Set position reference
double theta_ref = 1.0; // rad

// Solve position LQR
computeOptimalGain posLQR(sys, wf_pos, true);
MatrixXd Kp = posLQR.solveLQR();

// Create controller
OptimalPIDController controller(Kp, wf_pos, Ts, umax, kr_pos);

// Control loop
Vector3d x_hat = observer.state();
controller.updateState(x_hat);
double u = controller.computeControl(theta_ref, theta_meas);
```

## Troubleshooting

### Common Issues

1. **Compilation errors with Eigen**:
   - Ensure Eigen is in include path: `-I/path/to/eigen`
   - Check C++17 standard: `-std=c++17`

2. **Observer divergence**:
   - Adjust observer gain matrix `L_`
   - Check measurement noise levels

3. **RLS instability**:
   - Increase initial covariance `P_`
   - Add forgetting factor if needed

4. **Control saturation**:
   - Increase `umax` if motor can handle it
   - Reduce reference magnitudes
   - Tune LQR weights `Q` and `R`

5. **Poor tracking performance**:
   - Tune LQR weights (increase position/velocity weights in `Q`)
   - Adjust filter frequencies `wf_vel`, `wf_pos`
   - Check observer accuracy
   

## Author

[Your name/team]

## Version History

- **v1.0**: Initial implementation with observer, RLS, and LQR control


#include <iostream>
#include <limits>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

//------------------------
// State-space representation
// x = [theta_out; omega_in; i]
// y = [theta_out; omega_in]
//------------------------
struct stateSpaceRep {
    Matrix3d      Ad;               // 3x3
    Vector3d      Bd;               // 3x1 (control: voltage)
    Vector3d      Ed;               // 3x1 (disturbance: load torque, eq. at motor)
    Eigen::Matrix<double,2,3> C;    // 2x3 (theta_out, omega_in)
};

//------------------------
// Create and discretize state space matrices
// Ts      : sampling period
// N_gear  : gear ratio (omega_in = N_gear * omega_out)
// B_eq    : equivalent viscous damping (referred to motor side)
// J_eq    : equivalent inertia (referred to motor side)
// Ke, R, L, Kt : electrical/motor params
//------------------------
stateSpaceRep formDiscreteMatrices(const double Ts, const double N_gear, const double B_eq, const double J_eq, const double Ke, const double R, const double L, const double Kt)
{
    stateSpaceRep sys;

    // Continuous-time A for x = [theta_out; omega_in; i]
    //
    // theta_out_dot = omega_out = (1/N_gear)*omega_in
    // omega_in_dot  = -(B_eq/J_eq)*omega_in + (Kt/J_eq)*i + (1/J_eq)*tau_load_eq
    // i_dot         = -(Ke/L)*omega_in      - (R/L)*i     + (1/L)*v
    //
    Matrix3d A;
    A << 0.0,        1.0 / N_gear,    0.0,
         0.0,       -B_eq / J_eq,     Kt / J_eq,
         0.0,       -Ke  / L,        -R  / L;

    Vector3d B;  // input: voltage v
    B << 0.0,
         0.0,
         1.0 / L;

    Vector3d E;  // disturbance: equivalent load torque tau_load_eq
    E << 0.0,
         1.0 / J_eq,
         0.0;

    // Output: y = [theta_out; omega_in]
    sys.C << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0;

    // Discretize A, B, and E using block-matrix exponential
    MatrixXd B_all(3, 2);
    B_all.col(0) = B;  // voltage
    B_all.col(1) = E;  // load torque

    MatrixXd zerosC = MatrixXd::Zero(2, 3);
    MatrixXd zerosD = MatrixXd::Zero(2, 2);

    MatrixXd block(3 + 2, 3 + 2);
    block.setZero();

    // [ A  B_all ]
    // [ 0    0   ]
    block.topLeftCorner(3, 3) = A;
    block.topRightCorner(3, 2) = B_all;
    block.bottomLeftCorner(2, 3) = zerosC;
    block.bottomRightCorner(2, 2) = zerosD;

    MatrixXd disc = (block * Ts).exp();

    // Extract discrete A, Bd, Ed
    sys.Ad = disc.topLeftCorner<3,3>();

    MatrixXd B_alld = disc.topRightCorner(3, 2);
    sys.Bd = B_alld.col(0);  // voltage
    sys.Ed = B_alld.col(1);  // load torque

    return sys;
}

//------------------------
// Luenberger observer
// xhat_{k+1} = Ad xhat_k + Bd u_k + L (y_k - C xhat_k)
// y = [theta_out; omega_in]
// L is 3x2
//------------------------
class LuenbergerObserver {
public:
    explicit LuenbergerObserver(const stateSpaceRep& sys)
        : sys_(sys)
    {
        L_ << 0.0, 0.0,
              0.0, 0.0,
              0.1, 0.1; 
        state_est_.setZero();
    }

    // Step state estimate forward
    // y_meas = [theta_out_meas; omega_in_meas]
    void step(const Vector2d& y_meas, double u) {
        Vector2d prediction = sys_.C * state_est_;
        Vector2d innovation = y_meas - prediction;
        state_est_ = sys_.Ad * state_est_ + sys_.Bd * u + L_ * innovation;
    }

    const Vector3d& state() const {
        return state_est_;
    }

private:
    stateSpaceRep sys_;
    Eigen::Matrix<double,3,2> L_;  // observer gain
    Vector3d state_est_;           // [theta_out_hat; omega_in_hat; i_hat]
};

//------------------------
// RLS Solver for Mechanical Params
// Uses omega_in and its derivative
//------------------------
class MechanicalRLS {
public:
    MechanicalRLS()
        : theta_(Vector3d::Zero()),
          reg_(Vector3d::Zero()),
          P_(1e6 * Matrix3d::Identity())
    {}

    // wdot: derivative of omega_in (rad/s^2)
    // w   : omega_in (rad/s)
    // i   : current (A)
    // tauL_eq: equivalent load torque at motor (N·m)
    void update(double wdot, double w, double i, double tauL_eq)
    {
        // Regression vector ϕ = [ i, -w, -tauL_eq ]^T
        reg_ << i, -w, -tauL_eq;

        // Prediction
        double y_hat = reg_.dot(theta_);

        // Gain K(k) = Pϕ / (1 + ϕᵀ P ϕ)
        double denom = 1.0 + reg_.transpose() * P_ * reg_;
        Vector3d K = (P_ * reg_) / denom;

        // Parameter update
        theta_ = theta_ + K * (wdot - y_hat);

        // Covariance update
        P_ = P_ - K * reg_.transpose() * P_;
    }

    // theta_ = [Kt/J_eq, B_eq/J_eq, 1/J_eq]
    double getJ() const {
        double a3 = theta_(2); // = 1/J_eq
        if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
        return 1.0 / a3;
    }

    double getB() const {
        double a3 = theta_(2); // = 1/J_eq
        if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
        double J = 1.0 / a3;
        double a2 = theta_(1); // = B_eq/J_eq
        return a2 * J;
    }

    double getKt() const {
        double a3 = theta_(2); // = 1/J_eq
        if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
        double J = 1.0 / a3;
        double a1 = theta_(0); // = Kt/J_eq
        return a1 * J;
    }

    const Vector3d& rawTheta() const { return theta_; }
    const Matrix3d& covariance() const { return P_; }

private:
    Vector3d theta_;  // [Kt/J_eq, B_eq/J_eq, 1/J_eq]
    Vector3d reg_;    // [i, -w, -tauL_eq]
    Matrix3d P_;
};

//------------------------
// RLS Solver for Electrical Params
// Uses current and its derivative
//------------------------
class ElectricalRLS {
public:
    ElectricalRLS()
        : theta_(Vector3d::Zero()),
          reg_(Vector3d::Zero()),
          P_(1e6 * Matrix3d::Identity())
    {}

    // idot: derivative of i (A/s)
    // i   : current (A)
    // w   : omega_in (rad/s)
    // V   : applied voltage (V)
    void update(double idot, double i, double w, double V)
    {
        // Regression vector ϕ = [ i, w, V ]^T
        reg_ << i, w, V;

        // Prediction
        double y_hat = reg_.dot(theta_);

        // Gain K(k) = Pϕ / (1 + ϕᵀ P ϕ)
        double denom = 1.0 + reg_.transpose() * P_ * reg_;
        Vector3d K = (P_ * reg_) / denom;

        // Parameter update
        theta_ = theta_ + K * (idot - y_hat);

        // Covariance update
        P_ = P_ - K * reg_.transpose() * P_;
    }

    // theta_ = [-R/L, -Ke/L, 1/L]
    double getL() const {
        double a3 = theta_(2); // = 1/L
        if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
        return 1.0 / a3;
    }

    double getR() const {
        double a3 = theta_(2); // = 1/L
        if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
        double L = 1.0 / a3;
        double a1 = theta_(0); // = -R/L
        return -a1 * L;
    }

    double getKe() const {
        double a3 = theta_(2); // = 1/L
        if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
        double L = 1.0 / a3;
        double a2 = theta_(1); // = -Ke/L
        return -a2 * L;
    }

    const Vector3d& rawTheta() const { return theta_; }
    const Matrix3d& covariance() const { return P_; }

private:
    Vector3d theta_;  // [-R/L, -Ke/L, 1/L]
    Vector3d reg_;    // [i, w, V]
    Matrix3d P_;
};

//------------------------
// Solve LQR
//------------------------
class computeOptimalGain {
    public:
        computeOptimalGain(stateSpaceRep& sys, double wf, bool mode)
        :Aa(5,5),
        Ba(5,1),
        Ea(5,1),
        Q(5,5),
        R(1,1),
        P(5,5),
        K(1,5),
        sys_(sys),
        wf_(wf)
        {
            //Set Cost Matrices
            VectorXd q_diag(5);
            q_diag << 1, 1, 1, 1, 1;
            Q = q_diag.asDiagonal();
            R << 1;

            // Augment State Matrix
            Aa.setZero();
            Aa.topLeftCorner(3,3) = sys_.Ad;
            MatrixXd idstack(2, 3);
            int mode_int = mode ? 1 : 0;
            idstack.row(0) = -sys_.C.row(mode_int); //mode = 0 gives w, 1 gives theta
            idstack.row(1) = -wf_ * sys_.C.row(mode_int);
            Aa.bottomLeftCorner(2,3) = idstack;
            MatrixXd bot_right(2,2);
            bot_right << 0, 0,
            0, -wf_;
            Aa.bottomRightCorner(2, 2) = bot_right;

            //Augment Input Matrix
            Ba.setZero();
            Ba.topRows(3) = sys_.Bd;

            //Augment Reference Matrix
            Ea << 0, 0, 0, 1, wf_;
            Ea = Ea.transpose().eval();
        }

        //solve LQR
        MatrixXd solveLQR(){
            solveDARE();
            K = (R + Ba.transpose() * P * Ba).inverse() * Ba.transpose() * P * Aa;
            return K;
        }
    private:
        void solveDARE(){
            MatrixXd A_(5,5) = Aa;
            MatrixXd G_(5,5) = Ba*R.inverse()*Ba.transpose();
            MatrixXd H_(5,5) = Q;
            MatrixXd A_next(5,5) = A_;
            MatrixXd G_next(5,5) = G_;
            MatrixXd H_next(5,5) = H_;
            MatrixXd I = MatrixXd::Identity(5,5);
            do{
                A_ = A_next;
                A_next = A_*(I + G_ * H_).inverse() * A_;
                G_ = G_next;
                G_next = G_ + A_*(I + G_ * H_).inverse() * G_ * A_.transpose();
                H_ = H_next;
                H_next = H_ + A_.transpose() * H_ * (I + G_ * H_).inverse() * A_;
            }
            while((H_next - H_).norm()/(H_next.norm()) >= 1e-6);
            P = (H_next + H_next.transpose())/2; //ensure matrix is symmetric by adding transpose and dividing
            return;
        }
    MatrixXd Aa;
    MatrixXd Ba;
    MatrixXd Ea;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd P;
    MatrixXd K;
    int row;
    stateSpaceRep sys_;
    double wf_;
};


//------------------------
// Velocity LQR Solver (PI-like: 4 states = 3 original + 1 integral)
//------------------------
class VelocityLQRSolver {
public:
    VelocityLQRSolver(stateSpaceRep& sys, double wf)
        : Aa(4, 4),
          Ba(4, 1),
          Ea(4, 1),
          Q(4, 4),
          R(1, 1),
          P(4, 4),
          K(1, 4),
          sys_(sys),
          wf_(wf)
    {
        // Set Cost Matrices
        VectorXd q_diag(4);
        q_diag << 1, 1, 1, 1;
        Q = q_diag.asDiagonal();
        R << 1;

        // Augment State Matrix for velocity control with integral
        // xa = [theta_out; omega_in; i; e_int] where e_int is integral of (omega_ref - omega_in)
        Aa.setZero();
        Aa.topLeftCorner(3, 3) = sys_.Ad;
        // Integral error dynamics: e_int(k+1) = e_int(k) - omega_in(k) + omega_ref
        // In augmented form: e_int(k+1) = e_int(k) - C.row(1) * x(k)
        // omega_in is at index 1, C.row(1) = [0, 1, 0]
        Aa.bottomLeftCorner(1, 3) = -sys_.C.row(1); // -[0, 1, 0]
        Aa(3, 3) = 1.0; // Integral state persists

        // Augment Input Matrix
        Ba.setZero();
        Ba.topRows(3) = sys_.Bd;
        Ba(3, 0) = 0.0; // Integral doesn't directly depend on input

        // Augment Reference Matrix (handles omega_ref input)
        Ea << 0, 0, 0, 1;
    }

    MatrixXd solveLQR() {
        solveDARE();
        K = (R + Ba.transpose() * P * Ba).inverse() * Ba.transpose() * P * Aa;
        return K;
    }

private:
    void solveDARE() {
        MatrixXd A_(4, 4) = Aa;
        MatrixXd G_(4, 4) = Ba * R.inverse() * Ba.transpose();
        MatrixXd H_(4, 4) = Q;
        MatrixXd A_next(4, 4) = A_;
        MatrixXd G_next(4, 4) = G_;
        MatrixXd H_next(4, 4) = H_;
        MatrixXd I = MatrixXd::Identity(4, 4);
        do {
            A_ = A_next;
            A_next = A_ * (I + G_ * H_).inverse() * A_;
            G_ = G_next;
            G_next = G_ + A_ * (I + G_ * H_).inverse() * G_ * A_.transpose();
            H_ = H_next;
            H_next = H_ + A_.transpose() * H_ * (I + G_ * H_).inverse() * A_;
        } while ((H_next - H_).norm() / (H_next.norm()) >= 1e-6);
        P = (H_next + H_next.transpose()) / 2;
    }

    MatrixXd Aa;
    MatrixXd Ba;
    MatrixXd Ea;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd P;
    MatrixXd K;
    stateSpaceRep sys_;
    double wf_;
};

//------------------------
// Velocity PI Controller (using LQR gain)
//------------------------
class VelocityPIController {
private:
    MatrixXd Kv;      // Velocity LQR gain matrix (1x4)
    VectorXd xa;      // Augmented state [theta_out; omega_in; i; e_int]
    double kr;        // Reference prefilter gain
    double Ts;        // Sampling period
    double umax;      // Actuator saturation limit

public:
    VelocityPIController(MatrixXd K, double sample, double sat, double gain)
        : Kv(K),
          xa(4),
          kr(gain),
          Ts(sample),
          umax(sat)
    {
        xa.setZero();
    }

    double computeControl(double omega_ref, double omega_meas) {
        // Update augmented state
        double error = omega_ref - omega_meas;
        xa(3) += Ts * error; // Integral (index 3)

        // Compute control: u = kr * r - Kv * xa
        double u = kr * omega_ref - (Kv * xa)(0);

        // Apply saturation
        return std::clamp(u, -umax, umax);
    }

    void updateState(const Vector3d& x) {
        xa.head(3) = x;
    }
};

//------------------------
// State-feedback controller (LQR-based PID-like)
//------------------------
class OptimalPIDController {
    private:
        MatrixXd Ka;      // Optimal gain matrix
        VectorXd xa;      // Augmented state
        double kr;        // Reference prefilter gain
        double omegaf;    // Derivative filter frequency
        double Ts;        // Sampling period
        double umax;      // Actuator saturation limit
    
    public:
        OptimalPIDController(MatrixXd K, double freq, double sample, double sat, double gain)
        :Ka(K),
        xa(5),
        kr(gain),
        omegaf(freq),
        Ts(sample),
        umax(sat)
        {}
        double computeControl(double reference, double measurement) {
            // Update augmented state
            double error = reference - measurement;
            xa(3) += Ts * error;                                   // Integral (index 3)
            xa(4) = (1 - omegaf * Ts) * xa(4) + omegaf * Ts * error; // Filtered derivative (index 4)
    
            // Compute control: u = kr * r - K * xa
            double u = kr * reference - (Ka * xa)(0);
    
            // Apply saturation
            return std::clamp(u, -umax, umax);
        }
        
        void updateState(const Vector3d& x) {
            xa.head(3) = x;
        }
    };

//------------------------
// Motor plant simulator
// x = [theta_out; omega_in; i]
//------------------------
class MotorPlant {
public:
    explicit MotorPlant(const stateSpaceRep& sys)
        : sys_(sys)
    {
        x_.setZero();
    }

    // Advance plant state one step with control input u_k (voltage)
    // and equivalent load torque tauL_eq at the motor side
    void step(double u, double tauL_eq = 0.0) {
        x_ = sys_.Ad * x_ + sys_.Bd * u + sys_.Ed * tauL_eq;
    }

    // Outputs: [theta_out; omega_in]
    Vector2d outputs() const {
        return sys_.C * x_;
    }

    double thetaOut() const { return x_(0); }
    double omegaIn() const { return x_(1); }
    double current()  const { return x_(2); }

    const Vector3d& state() const {
        return x_;
    }

private:
    stateSpaceRep sys_;
    Vector3d x_;   // [theta_out; omega_in; i]
};

//------------------------
// Example main()
//------------------------
#ifndef MOTOR_CONTROL_NO_MAIN
int main() {
    // Motor + gearbox initial guesses
    double B_eq     = 6e-5;   // equivalent damping (motor side)
    double J_eq     = 7e-7;   // equivalent inertia (motor side)
    double Ke       = 0.036;
    double R        = 2.6;
    double L        = 0.003;
    double Kt       = 0.036;
    double N_gear   = 50.0;   // motor : output

    double Ts       = 1e-4;   // Sampling period [s]
    double simTime  = 0.05;   // Simulation length [s]
    int    N        = static_cast<int>(simTime / Ts);

    // Form discrete state space matrices
    stateSpaceRep sys = formDiscreteMatrices(Ts, N_gear, B_eq, J_eq, Ke, R, L, Kt);

    // Initialize observer
    LuenbergerObserver observer(sys);

    // Initialize RLS estimators
    MechanicalRLS mechRLS;
    ElectricalRLS elecRLS;

    // Initialize plant (for simulation)
    MotorPlant plant(sys);

    // Velocity set point
    double omega_ref = 10.0; // rad/s

    // Speed scaling factor
    double speed_scale = 1.0;

    // Filter frequency for derivative
    double wf_vel = 100.0;  // Hz for velocity loop
    double wf_pos = 50.0;   // Hz for position loop

    // Control saturation
    double umax = 12.0; // V

    // Reference prefilter gains
    double kr_vel = 1.0;
    double kr_pos = 1.0;

    // Initialize controllers (will be updated after LQR solve)
    VelocityPIController* velController = nullptr;
    OptimalPIDController* posController = nullptr;

    // Variables for derivative estimation
    double omega_in_prev = 0.0;
    double i_prev = 0.0;
    double u_prev = 0.0;

    // Main control loop
    for (int k = 0; k < N; ++k) {
        // Get plant outputs (simulated measurements)
        Vector2d y_meas = plant.outputs();
        double theta_out_meas = y_meas(0);
        double omega_in_meas = y_meas(1);
        double i_meas = plant.current();

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
            double omega_in_dot = (omega_in_hat - omega_in_prev) / Ts;
            double i_dot = (i_hat - i_prev) / Ts;

            // Update mechanical RLS (assuming zero load torque for now)
            double tauL_eq = 0.0;
            mechRLS.update(omega_in_dot, omega_in_hat, i_hat, tauL_eq);

            // Update electrical RLS
            elecRLS.update(i_dot, i_hat, omega_in_hat, u_prev);

            // Step 3: Update state estimate (already done by observer)
            // Could update system parameters from RLS here for adaptive control

            // Step 4: Solve LQR for optimal gain matrices
            // Solve velocity LQR (PI-like)
            VelocityLQRSolver velLQR(sys, wf_vel);
            MatrixXd Kv = velLQR.solveLQR();

            // Solve position LQR (PID-like)
            computeOptimalGain posLQR(sys, wf_pos, true); // true = position mode
            MatrixXd Kp = posLQR.solveLQR();

            // Step 5: Create/update controllers
            if (velController != nullptr) {
                delete velController;
            }
            velController = new VelocityPIController(Kv, Ts, umax, kr_vel);

            if (posController != nullptr) {
                delete posController;
            }
            posController = new OptimalPIDController(Kp, wf_pos, Ts, umax, kr_pos);

            // Update previous values for next RLS update
            omega_in_prev = omega_in_hat;
            i_prev = i_hat;
        } else {
            // Normal step: update observer once
            observer.step(y_meas, u_prev);
        }

        // Step 6: Compute control given velocity set point
        Vector3d x_hat = observer.state();
        
        if (velController != nullptr) {
            velController->updateState(x_hat);
            
            // Apply speed scaling
            double omega_ref_scaled = speed_scale * omega_ref;
            
            // Compute velocity control (PI loop)
            double u_vel = velController->computeControl(omega_ref_scaled, x_hat(1));
            
            // Apply control to plant
            plant.step(u_vel);
            u_prev = u_vel;
        } else {
            // No control yet, just update plant with zero input
            plant.step(0.0);
            u_prev = 0.0;
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

    return 0;
}
#endif // MOTOR_CONTROL_NO_MAIN

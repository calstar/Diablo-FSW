#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

//------------------------
// Create and discretize state space matrices
//------------------------
struct stateSpaceRep {
    MatrixXd Ad;
    VectorXd Bd;
    VectorXd Ed;
    RowVector3d C;
};

stateSpaceRep formDiscreteMatrices(const double Ts, const double B_visc, const double J, const double Ke, const double R, const double L, const double Kt) {
    stateSpaceRep sys;

    MatrixXd A(3,3);
    A << 0, 1, 0,
    0, -B_visc/J, Kt/J,
    0, -Ke/L, -R/L;

    VectorXd B(3);
    B << 0,
    0,
    1/L;

    VectorXd E(3);
    E << 0,
    1/J,
    0;

    sys.C << 0, 1, 0;

    //Discretize A, B, and E
    MatrixXd B_all(3, 2);
    B_all.col(0) = B;
    B_all.col(1) = E;

    MatrixXd zerosC(B_all.cols(), A.cols());
    zerosC.setZero();

    MatrixXd zerosD(B_all.cols(), B_all.cols());
    zerosD.setZero();

    MatrixXd block(A.cols() + B_all.cols(), A.cols() + B_all.cols());
    block.topLeftCorner(A.rows(), A.cols()) = A;
    block.topRightCorner(B_all.rows(), B_all.cols()) = B_all;
    block.bottomLeftCorner(B_all.cols(), A.cols()) = zerosC;
    block.bottomRightCorner(B_all.cols(),B_all.cols()) = zerosD;
    
    MatrixXd disc = (block * Ts).exp();
    
    //extract Ad and Bd
    sys.Ad = disc.topLeftCorner(3,3);
    
    MatrixXd B_alld = disc.topRightCorner(3,2);

    sys.Bd = B_alld.col(0);
    sys.Ed = B_alld.col(1);
    return sys;
}


//------------------------
// Luenberger observer
//------------------------

class LuenbergerObserver {
    public:
    LuenbergerObserver(const stateSpaceRep& sys)
    :sys_(sys),
    L(Vector3d::Zero())
    state_est(Vector3d::Zero()),
    {
        L << 0.1,
        0.1,
        0.1;
    }
    void step(const double meas, const double u){   //step state estimate forward
        double prediction = (sys_.C * state_est)(0);
        double innovation = meas - prediction;
        state_est = sys_.Ad * state_est + sys_.Bd * u + L * (innovation);
        return;
    }
    private:
    stateSpaceRep sys_;
    Vector3d L;
    Vector3d state_est;
};

//------------------------
// Recursive Least Squares Solver
//------------------------

class RLS {
    public:
    RLS(stateSpaceRep& sys, int row) 
        :sys_(sys),
         reg(VectorXd::Zero(4)),
         param(VectorXd::Zero(4)),
         P(1e6 * MatrixXd::Identity(4,4)),
         gain(VectorXd::Zero(4))
         row(row)
    {
        param << sys_.Ad(row, 0), sys_.Ad(row, 1), sys_.Ad(row, 2), sys_.Bd(row); //get initial param estimates from state matrix
    }
    void update(double readings, double theta, double w, double i, double V) {   //calculate parameters, gain, and covariance
        reg << theta, w, i, V;
        computeGain();
        param = param + gain * (readings - predict());
        computeCovariance();
        return;
    }
    private:
    int row
    stateSpaceRep sys_;
    VectorXd param;
    MatrixXd P;
    VectorXd reg;
    VectorXd gain;
    double J, B, R, L, Kt, Ke;
    double predict(){
        return param.dot(reg);
    }
    void computeGain(){
        double adjust = 1 + reg.dot(P*reg);
        gain = P * reg / adjust;
        return;
        }
    void computeCovariance(){
        P = P - gain * reg.transpose() * P;
        return;
    }
};


//------------------------
// RLS Solver for Physical Params
//------------------------

class MechanicalRLS {
    public:
        MechanicalRLS()
            : theta_(Eigen::Vector3d::Zero()),
              reg_(Eigen::Vector3d::Zero()),
              P_(1e6 * Eigen::Matrix3d::Identity())
        {}
    
        // wdot: derivative of omega (rad/s^2)
        // w   : omega (rad/s)
        // i   : current (A)
        // tauL: load torque (N·m)  (if you don't know tauL, pass 0 and treat that term as disturbance)
        void update(double wdot, double w, double i, double tauL)
        {
            // Regression vector ϕ = [ i, -w, -tauL ]^T
            reg_ << i, -w, -tauL;
    
            // Prediction
            double y_hat = reg_.dot(theta_);
    
            // Gain K(k) = Pϕ / (1 + ϕ^T P ϕ)
            double denom = 1.0 + reg_.transpose() * P_ * reg_;
            Eigen::Vector3d K = (P_ * reg_) / denom;
    
            // Parameter update
            theta_ = theta_ + K * (wdot - y_hat);
    
            // Covariance update
            P_ = P_ - K * reg_.transpose() * P_;
        }
    
        // --- getters for physical parameters ---
        double getJ() const {
            double a3 = theta_(2); // = 1/J
            if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
            return 1.0 / a3;
        }
    
        double getB() const {
            double a3 = theta_(2); // = 1/J
            if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
            double J = 1.0 / a3;
            double a2 = theta_(1); // = B/J
            return a2 * J;
        }
    
        double getKt() const {
            double a3 = theta_(2); // = 1/J
            if (std::abs(a3) < 1e-12) return std::numeric_limits<double>::quiet_NaN();
            double J = 1.0 / a3;
            double a1 = theta_(0); // = Kt/J
            return a1 * J;
        }
    
        const Eigen::Vector3d& rawTheta() const { return theta_; }
        const Eigen::Matrix3d& covariance() const { return P_; }
    
    private:
        Eigen::Vector3d theta_;  // [Kt/J, B/J, 1/J]
        Eigen::Vector3d reg_;    // [i, -w, -tauL]
        Eigen::Matrix3d P_;
};
    

//------------------------
// RLS Solver for Electrical Params
//------------------------

class ElectricalRLS {
    public:
        ElectricalRLS()
            : theta_(Eigen::Vector3d::Zero()),
              reg_(Eigen::Vector3d::Zero()),
              P_(1e6 * Eigen::Matrix3d::Identity())
        {}
    
        // idot: derivative of current (A/s)
        // i   : current (A)
        // w   : omega (rad/s)
        // V   : applied voltage (V)
        void update(double idot, double i, double w, double V)
        {
            // Regression vector ϕ = [ i, w, V ]^T
            reg_ << i, w, V;
    
            // Prediction
            double y_hat = reg_.dot(theta_);
    
            // Gain K(k) = Pϕ / (1 + ϕ^T P ϕ)
            double denom = 1.0 + reg_.transpose() * P_ * reg_;
            Eigen::Vector3d K = (P_ * reg_) / denom;
    
            // Parameter update
            theta_ = theta_ + K * (idot - y_hat);
    
            // Covariance update
            P_ = P_ - K * reg_.transpose() * P_;
        }
    
        // --- getters for physical parameters ---
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
    
        const Eigen::Vector3d& rawTheta() const { return theta_; }
        const Eigen::Matrix3d& covariance() const { return P_; }
    
    private:
        Eigen::Vector3d theta_;  // [-R/L, -Ke/L, 1/L]
        Eigen::Vector3d reg_;    // [i, w, V]
        Eigen::Matrix3d P_;
};
    
//------------------------
// LQR Solver
//------------------------


//------------------------
// State-feedback controller
//------------------------

class OptimalPIDController {
    private:
        Eigen::MatrixXd Ka;   // Optimal gain matrix (from LQR)
        Eigen::VectorXd xa;   // Augmented state = [x; xI; xD]
        double kr;            // Reference prefilter gain
        double omegaf;        // Derivative filter frequency
        double Ts;            // Sampling period
        double u_max;         // Actuator saturation limit
    
    public:
        // Compute control input u(k)
        double computeControl(double reference, double measurement) {
    
            // --- Compute tracking error ---
            double error = reference - measurement;
    
            // --- Update augmented states ---
            xa(1) += Ts * error;  // Integral state
    
            // Filtered derivative state:  xD(k+1) = (1 - ωf*Ts)xD + ωf*Ts*e
            xa(2) = (1.0 - omegaf * Ts) * xa(2) + omegaf * Ts * error;
    
            // --- Compute raw control ---
            // u = k_r * r - Kᵀ * x_a
            double u = kr * reference - Ka.transpose() * xa;
    
            // --- Apply saturation ---
            return std::clamp(u, -u_max, u_max);
        }
};    


//------------------------
// Motor plant simulator
//------------------------

class MotorPlant {
    public:
        explicit MotorPlant(const stateSpaceRep& sys)
            : sys_(sys)
        {
            x_.setZero();
        }
    
        // Advance plant state one step with control input u_k
        void step(double u) {
            x_ = sys_.Ad * x_ + sys_.Bd * u;
        }
    
        // Output y_k = C x_k
        double output() const {
            return (sys_.C * x_)(0);  // scalar
        }
    
        const Vector3d& state() const {
            return x_;
        }
    
    private:
        stateSpaceRep sys_;
        Vector3d x_;
    };

void main() {
    // Motor parameters (your initial guesses)
    double B_visc = 6e-5;
    double J      = 7e-7;
    double Ke     = 0.036;
    double R      = 2.6;
    double Kt     = 0.036;

    double Ts = 1e-4;   // Sampling period [s]
    double simTime = 0.05; // Simulation length [s]
    int N = static_cast<int>(simTime / Ts);

    return;
}

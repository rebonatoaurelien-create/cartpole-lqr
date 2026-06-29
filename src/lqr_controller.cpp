/**
 * @file lqr_controller.cpp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */

#include <cmath>
#include <Eigen/Dense>
#include <iostream>

#include <lqr_controller.hpp>

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;


MatrixXd LQRController::computeK(const Matrix4d& A, const Vector4d& B, const MatrixXd& R, const MatrixXd& Q, double dt) const
{   
    Matrix4d S = Q; // Initial guess for S
    auto riccati_lambda = [&](double t, const MatrixXd& S) -> MatrixXd 
    {
        return riccatiEquation(S, A, B, R, Q);
    };
    S = rk4MatrixSolver(Q, riccati_lambda, -dt);   
    MatrixXd K = R.inverse() * B.transpose() * S;
    return K;
}
double LQRController::computeControl(const MatrixXd & K, const Vector4d& X_current, const Vector4d& X_target) const
{
    MatrixXd u_matrix = -K * (X_current - X_target);
    double u = u_matrix(0,0);
    return std::max(-100.0, std::min(100.0, u));
    
}
MatrixXd LQRController::riccatiEquation(const Matrix4d S,const Matrix4d& A, const Vector4d& B, const MatrixXd& R, const MatrixXd& Q) const
{
    // dS/dt = -S A - A^T S + S B R^-1 B^T S - Q
    return -S * A - A.transpose() * S + S * B * R.inverse() * B.transpose() * S - Q;
}
MatrixXd LQRController::rk4MatrixStep(const std::function<MatrixXd(double t,const MatrixXd&)>& f,
                const MatrixXd& S_current,
                double dt) const
{
    double t {0};
    MatrixXd k1 = f(t, S_current);
    MatrixXd k2 = f(t,S_current + k1 * (dt / 2.0));
    MatrixXd k3 = f(t,S_current + k2 * (dt / 2.0));
    MatrixXd k4 = f(t,S_current + k3 * dt);
    return S_current + (dt/6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
MatrixXd LQRController::rk4MatrixSolver(const MatrixXd S_initial_guess, const std::function<MatrixXd(double, const MatrixXd&)>& f, double dt) const
{    
    int max_iterations {5000};
    double max_error {1e-4};    MatrixXd S = S_initial_guess;    
    for (int i = 0; i < max_iterations; ++i)
    {
        MatrixXd S_next = rk4MatrixStep(f, S, dt);
            if ((S_next - S).cwiseAbs().maxCoeff() < max_error)
            {
                return S_next;
            }
        S = S_next;
    }
    std::cerr << "Warning: The CARE solver was unable to converge" << std::endl;
    return S;
}        

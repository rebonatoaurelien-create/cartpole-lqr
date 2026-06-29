/**
 * @file lqr_controller.hpp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */
#pragma once

#include <Eigen/Dense>

using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

class LQRController
{
    public:

    /**
     * @brief Computes the control gain matrix K
     * 
     * @param A State Jacobian matrix
     * @param B Control Jacobian Matrix
     * @param R Control cost matrix
     * @param Q State cost matrix
     * @param dt Time step for the RK4 integration
     * @return MatrixXd Optimal feedback gain matrix K.
     */
        MatrixXd computeK(const Matrix4d& A, const Vector4d& B, const MatrixXd& R, const MatrixXd& Q, double dt) const;
        
        /**
         * @brief Computes the control input u based on the LQR control law
         * 
         * @param K Optimal feedback gain matrix
         * @param X_current Current state vector [x, theta, dx, dtheta]
         * @param X_target Target state vector [x, theta, dx, dtheta]
         * @return double Control: force in N
         */
        double computeControl(const MatrixXd & K, const Vector4d& X_current, const Vector4d& X_target) const;
        
        /**
         * @brief Evaluates the Continuous Algebraic Riccati Equation (CARE).
         * @param S Current solution estimate.
         * @param A State matrix.
         * @param B Input matrix.
         * @param R Control cost.
         * @param Q State cost.
         * @return MatrixXd The derivative dS/dt.
         */
        MatrixXd riccatiEquation(const Matrix4d S,const Matrix4d& A, const Vector4d& B, const MatrixXd& R, const MatrixXd& Q) const;
        
        /**
         * @brief Performs a single Runge-Kutta 4 step for matrix integration.
         * @param f Lambda function representing the differential equation.
         * @param S_current The matrix state at the current step.
         * @param dt Integration time step.
         * @return MatrixXd The matrix state at the next step.
         */
        MatrixXd rk4MatrixStep(const std::function<MatrixXd(double t,const MatrixXd&)>& f,
                            const MatrixXd& S_current,
                            double dt) const;

        /**
         * @brief Solves the matrix differential equation using RK4 until convergence.
         * @param S_initial_guess Initial guess for the S matrix (usually Q).
         * @param f Lambda function representing the ODE.
         * @param dt Integration time step (should be negative for CARE).
         * @return MatrixXd The converged S matrix.
         */
        MatrixXd rk4MatrixSolver(const MatrixXd S_initial_guess, const std::function<MatrixXd(double, const MatrixXd&)>& f, double dt) const;

};

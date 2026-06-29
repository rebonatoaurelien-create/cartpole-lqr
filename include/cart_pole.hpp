/**
 * @file cart_pole.hpp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */

#pragma once

#include <Eigen/Dense>

using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;

/**
 * @brief Class that include the Cart-Pole charateristics and dynamics.
 * 
 */
class CartPole
{
    private:
        double length_;
        double c_mass_;
        double p_mass_;

        static constexpr double kGravity = 9.807;

    public:
        CartPole(double length, double c_mass, double p_mass);
        
        /**
         * @brief From the state and the force applied, compute the non-lienaer continuous time dynamics of the system (equation from Underactuated Robotics (Tedrake, 2018))
         * 
         * @param f Force applied to the cart (N)
         * @param X Current state vector [x, theta, dx, dtheta]
         * @return Vector4d State derivative vector dX:dt.
         */
        Vector4d cartPoleDynamics(double f, const Vector4d& X) const;
        
        /**
         * @brief Compute the state Jacobian matrix A using numerical central differences
         * @param X State vector around which to linearize
         * @param u Control input around which to linearize.
         * @return Matrix4d State Jacobian matrix A
         */
        Matrix4d computeJacobianA(const Vector4d& X, double u) const;
        
        /**
         * @brief Compute the control Jacobian matrix B using nuerical central differences 
         * 
         * @param X  State vector around which to linearize
         * @param u Control input around which to linearize
         * @return Vector4d Linearized input MAtrix B
         */
        Vector4d computeJacobianB(const Vector4d& X, double u) const;
};


/**
 * @file cart_pole.cpp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */

#include <cmath>
#include <Eigen/Dense>

#include <cart_pole.hpp>


using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;


CartPole::CartPole( 
            double m_length, 
            double m_c_mass, 
            double m_p_mass
            ) 
    {
        length_ = m_length;
        c_mass_ = m_c_mass; // Cart mass
        p_mass_ = m_p_mass; // Pendulum mass
    };

Vector4d CartPole::cartPoleDynamics(double f, const Vector4d& X) const
{
    // Compute the dynamics of the cart pole according to the equations of motion from Underactuated Robotics (Russ Tedrake).
    double pos = X(0);
    double theta = X(1);
    double vel = X(2);
    double d_theta = X(3);
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);
    double denominator = c_mass_ + p_mass_ * sin_theta * sin_theta;
    Vector4d dX;
    dX(0) = vel;
    dX(1) = d_theta;
    dX(2) = (f + p_mass_ * sin_theta * (length_ * d_theta * d_theta + kGravity * cos_theta))/denominator;
    dX(3) = (-f * cos_theta - p_mass_ * length_ * d_theta * d_theta * cos_theta * sin_theta - (c_mass_ + p_mass_) * kGravity * sin_theta)/(denominator * length_); 
    return dX;
}
Matrix4d CartPole::computeJacobianA(const Vector4d& X, double u) const
{
    Matrix4d A;
    double epsilon = 1e-6;
    for (int i = 0; i < 4; ++i) {
        Vector4d X_plus = X;
        Vector4d X_minus = X;
        X_plus(i) += epsilon;
        X_minus(i) -= epsilon;
        Vector4d f_plus = cartPoleDynamics(u, X_plus);
        Vector4d f_minus = cartPoleDynamics(u, X_minus);
        A.col(i) = (f_plus - f_minus) / (2.0*epsilon);
    }
    return A;
}
Vector4d CartPole::computeJacobianB(const Vector4d& X, double u) const
{
    Vector4d B;
    double epsilon = 1e-6;
    Vector4d f_plus = cartPoleDynamics(u + epsilon, X);
    Vector4d f_minus = cartPoleDynamics(u - epsilon, X);
    B = (f_plus - f_minus) / (2.0*epsilon);
    return B;
}

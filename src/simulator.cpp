/**
 * @file simulator.cpp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */

#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include <cart_pole.hpp>
#include <lqr_controller.hpp>
#include <simulator.hpp> 

Vector4d Simulator::rk4Step(const std::function<Vector4d(double, const Vector4d&)>& f, 
            double t, 
            const Vector4d& y, 
            double dt) 
{
    Vector4d k1 = f(t, y);
    Vector4d k2 = f(t + dt / 2.0, y + k1 * (dt / 2.0));
    Vector4d k3 = f(t + dt / 2.0, y + k2 * (dt / 2.0));
    Vector4d k4 = f(t + dt, y + k3 * dt);
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
void Simulator::runSimulation(const Vector4d& X_initial, const Vector4d& X_target, const CartPole& cart, const LQRController& controller, double t_max, double dt,  const MatrixXd& Q, const MatrixXd& R, std::string fileName) 
{
    std::ofstream file(fileName);
    if (!file.is_open())
    {
        std::cerr << "Error: The file cannot be opened" << std::endl;
    }
    file << "time,x,theta,dx,dtheta\n";
    double t {0};
    double u {0}; // At t = 0, the control is null
    Vector4d X = X_initial;
    Matrix4d A = cart.computeJacobianA(X_target, u);
    Vector4d B = cart.computeJacobianB(X_target, u);
    MatrixXd K = controller.computeK(A, B, R, Q, dt);
    while (t < t_max)
    {
        Vector4d X_current = X;
        file << t << "," << X(0) << "," << X(1) << "," << X(2) << "," << X(3) << "\n";
        
        u = controller.computeControl(K, X_current, X_target);
        
        auto cartPoleDynamics_lambda = [&](double t, const Vector4d& X) {
            return cart.cartPoleDynamics(u, X);
        };
        // Update the state
        X = rk4Step(cartPoleDynamics_lambda, t, X_current, dt);
        t += dt;
    }
    file.close();
}

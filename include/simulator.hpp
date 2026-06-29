/**
 * @file simulator.hpp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */

#pragma once

#include <Eigen/Dense>

#include <cart_pole.hpp>
#include <lqr_controller.hpp>

class Simulator
{
    private:
        static Vector4d rk4Step(const std::function<Vector4d(double, const Vector4d&)>& f, 
                    double t, 
                    const Vector4d& y, 
                    double dt);
    
    public:

        /**
         * @brief Simulate the system dynamics. Create a .csv file.
         * 
         * @param X_initial Initial state vector
         * @param X_target Target state vector
         * @param cart CartPole object
         * @param controller LQRController object
         * @param t_max Duration of the simulation
         * @param dt Interval between each computation
         * @param Q State cost matrix
         * @param R Control cost matrix
         * @param fileName output file name. Must end with .csv
         */
        void runSimulation(const Vector4d& X_initial, const Vector4d& X_target, const CartPole& cart, const LQRController& controller, double t_max, double dt,  const MatrixXd& Q, const MatrixXd& R, std::string fileName);
       
};
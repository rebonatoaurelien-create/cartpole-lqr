/**
 * @file main.pp
 * @author [REBONATO Aurélien]
 * @brief Implementation of the non-linear Cart-Pole physics.
 * @date 2026
 */

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <functional>
#include <fstream>
#include <Eigen/Dense>

#include <cart_pole.hpp>
#include <lqr_controller.hpp>
#include <simulator.hpp>

using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

const double g {9.807};
const double pi = std::acos(-1);


int main()
{

    // Initialization of the cart's charateristics
    double cart_mass {1};
    double pendulum_mass {1};
    double length {1};

    // Initial state and target state

    Vector4d X_init;
    X_init << 0,pi-1,0,0;
    Vector4d X_target;
    X_target << 0,pi,0,0;

    // Cost Matrices R and Q
    MatrixXd R(1,1);
    R << 0.9;

    Matrix4d Q;
    Q << 1, 0, 0, 0,
        0, 100, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    

    double dt {1e-3};
    double t {20};

    std::string fileName {"simulation.csv"};
    
    CartPole cart{length, cart_mass, pendulum_mass};
    LQRController controller;
    Simulator simulator;

    simulator.runSimulation(X_init, X_target, cart, controller, t,dt, Q, R, fileName);

    
    return 0;
}


#include "Kalman.h"
#include <math.h>

void Kalman::init(double x, double y, double theta, double v, double yaw) {
    m_x << x, y, theta, v, yaw;
    m_I.setIdentity();
    Eigen::Matrix<double, 5, 1> tmp;
    tmp << 100, 100, 1000, 1000, 1000;
    m_P = tmp.asDiagonal();
    ready = true;
}

Kalman::Kalman() {
}


void Kalman::predict(double dt) {
    Eigen::MatrixXd JA(5, 5);
    if (abs(m_x[4]) < 0.0001) {  // Driving straight
        m_x[0] = m_x[0] + m_x[3] * dt * cos(m_x[2]);
        m_x[1] = m_x[1] + m_x[3] * dt * sin(m_x[2]);
        m_x[2] = m_x[2];
        m_x[3] = m_x[3];
        m_x[4] = 0.0000001;//  # avoid numerical issues in Jacobians


        JA << 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, dt,
                0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0;
    } else {
        m_x[0] = m_x[0] + (m_x[3] / m_x[4]) * (sin(m_x[4] * dt + m_x[2]) - sin(m_x[2]));
        m_x[1] = m_x[1] + (m_x[3] / m_x[4]) * (-cos(m_x[4] * dt + m_x[2]) + cos(m_x[2]));
        m_x[2] = fmod((m_x[2] + m_x[4] * dt + M_PI), (2.0 * M_PI)) - M_PI;
        m_x[3] = m_x[3];
        m_x[4] = m_x[4];
        // Calculate the Jacobian of the Dynamic Matrim_x A
        // see "Calculate the Jacobian of the Dynamic Matrim_x with respect to the state vector"
        double a13 = (m_x[3] / m_x[4]) * (cos(m_x[4] * dt + m_x[2]) - cos(m_x[2]));
        double a14 = (1.0 / m_x[4]) * (sin(m_x[4] * dt + m_x[2]) - sin(m_x[2]));
        double a15 = (dt * m_x[3] / m_x[4]) * cos(m_x[4] * dt + m_x[2]) - (m_x[3] / (m_x[4] * m_x[4])) * (sin(m_x[4] * dt + m_x[2]) - sin(m_x[2]));
        double a23 = (m_x[3] / m_x[4]) * (sin(m_x[4] * dt + m_x[2]) - sin(m_x[2]));
        double a24 = (1.0 / m_x[4]) * (-cos(m_x[4] * dt + m_x[2]) + cos(m_x[2]));
        double a25 = (dt * m_x[3] / m_x[4]) * sin(m_x[4] * dt + m_x[2]) - (m_x[3] / (m_x[4] * m_x[4])) * (-cos(m_x[4] * dt + m_x[2]) + cos(m_x[2]));

        JA << 1.0, 0.0, a13, a14, a15,
                0.0, 1.0, a23, a24, a25,
                0.0, 0.0, 1.0, 0.0, dt,
                0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0;
    }


    double sGPS = 0.5 * 8.8 * dt * dt;  // assume 8.8m/s2 as maximum acceleration, forcing the vehicle
    double sCourse = 0.1 * dt;  // assume 0.1rad/s as maximum turn rate for the vehicle
    double sVelocity = 8.8 * dt;  // assume 8.8m/s2 as maximum acceleration, forcing the vehicle
    double sYaw = 1.0 * dt; // assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle

    Eigen::MatrixXd Q(5, 5);
    Q << sGPS * sGPS * 1000, 0.0, 0.0, 0.0, 0.0,
            0.0, sGPS * sGPS * 1000, 0.0, 0.0, 0.0,
            0.0, 0.0, sCourse * sCourse * 10, 0.0, 0.0,
            0.0, 0.0, 0.0, sVelocity * sVelocity, 0.0,
            0.0, 0.0, 0.0, 0.0, sYaw * sYaw;
    Q;

    // Project the error covariance ahead
    m_P = JA * m_P * JA.transpose() + Q;
}

void Kalman::update(double x, double y, double theta, double speed, double yaw) {
    double z2_1 = theta;
    double z2_2 = theta + 2 * M_PI;
    double z2_3 = theta - 2 * M_PI;

    if(abs(z2_1 - m_x[2])<abs(z2_2 - m_x[2]) && abs(z2_1 - m_x[2])<abs(z2_3 - m_x[2]))
        theta = z2_1;
    if(abs(z2_2 - m_x[2])<abs(z2_1 - m_x[2]) && abs(z2_2 - m_x[2])<abs(z2_3 - m_x[2]))
        theta = z2_2;
    if(abs(z2_3 - m_x[2])<abs(z2_1 - m_x[2]) && abs(z2_3 - m_x[2])<abs(z2_2 - m_x[2]))
        theta = z2_3;



    Eigen::Matrix<double, 5, 1> Z;
    Z << x, y, theta, speed, yaw;
    // Measurement Function

    Eigen::Matrix<double, 5, 1> hx;
    hx << m_x[0], m_x[1], m_x[2], m_x[3], m_x[4];

    Eigen::MatrixXd JH(5, 5);

    JH << 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0;

    double varGPS = 6.0;    // Standard Deviation of GPS Measurement
    double varspeed = 1.0;  // Variance of the speed measurement
    double varyaw = 0.1;    // Variance of the yawrate measurement
    double sCourse = 0.1;   // assume 0.1rad/s as maximum turn rate for the vehicle

    Eigen::Matrix<double, 5, 5> m_R;
//    m_R << varGPS * varGPS, 0.0, 0.0, 0.0,
//            0.0, varGPS * varGPS, 0.0, 0.0,
//            0.0, 0.0, varspeed * varspeed, 0.0,
//            0.0, 0.0, 0.0, varyaw * varyaw;

    m_R << varGPS * varGPS, 0.0, 0.0, 0.0, 0.0,
            0.0, varGPS * varGPS, 0.0, 0.0, 0.0,
            0.0, 0.0, sCourse * sCourse, 0.0, 0.0,
            0.0, 0.0, 0.0, varspeed * varspeed, 0.0,
            0.0, 0.0, 0.0, 0.0, varyaw * varyaw;
    Eigen::Matrix<double, 5, 5> S = JH * m_P * JH.transpose() + m_R;
    Eigen::Matrix<double, 5, 5> K = (m_P * JH.transpose()) * S.inverse();

    // Update the estimate via
    Eigen::Matrix<double, 5, 1> Y = Z - hx;  // Innovation or Residual
    m_x = m_x + (K * Y);

    // Update the error covariance
    m_P = (m_I - (K * JH)) * m_P;
}
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
    TODO:
      * predict the state
    */

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */

    VectorXd y = z - H_ * x_; //z_pred = H_ * x_
    MatrixXd Ht = H_.transpose();
    MatrixXd P_Ht = P_ * Ht;
    MatrixXd S = H_ * P_Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_Ht * Si;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    //new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float rho = sqrt(px * px + py * py);
    float phi = atan2(py, px); //Note atan2(0,0) is undefined
    float rho_dot;

    //checks division by zero
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = ((px * vx) + (py * vy)) / rho;
    }

    VectorXd z_radar(3);
    z_radar << rho, phi, rho_dot;

    VectorXd y = z - z_radar;
    y(1) = atan2(sin(y(1)), cos(y(1))); // angle normilization, explination https://discussions.udacity.com/t/ekf-gets-off-track/276122/18
    MatrixXd Ht = H_.transpose();
    MatrixXd P_Ht = P_*Ht;
    MatrixXd S = H_ *P_Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_Ht * Si;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    //new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;


}

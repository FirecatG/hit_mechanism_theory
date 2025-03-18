#include "four_bar_mechanism.h"
#include "ros/console.h"
#include <cmath>
#include <stdexcept>
FourBarMechanism::FourBarMechanism(double omega, double l_AB, double l_BC, 
                    double l_CD, double l_AD, double l_BE, double l_EF)
        : omega_(omega), l_AB_(l_AB), l_BC_(l_BC), l_CD_(l_CD),
          l_AD_(l_AD), l_BE_(l_BE), l_EF_(l_EF) ,rates_({0,0}) ,phi_{0,0,0} {
              
          }

FourBarMechanism::~FourBarMechanism() = default;

std::vector<Result> FourBarMechanism::calculate(double start_deg, double end_deg, double step_deg) {
        std::vector<Result> results;
        for (double phi = start_deg; phi <= end_deg; phi += step_deg) {

            results.push_back(calculate_step(phi));
            
        }
        return results;
}

Result FourBarMechanism::calculate_step(double phi_deg) {
    current_phi_ = phi_deg;  
    const double phi = deg2rad(phi_deg);

    // 计算B点坐标
    const Point B = {l_AB_ * cos(phi), l_AB_ * sin(phi)};
    
    // 计算C点坐标
    const Point D = {l_AD_, 0};
    const double A0 = 2 * l_BC_ * (D.x - B.x);
    const double B0 = 2 * l_BC_ * (D.y - B.y);
    const double l_BD = sqrt(pow(D.x - B.x, 2) + pow(D.y - B.y, 2));
    const double C0 = pow(l_BC_, 2) + pow(l_BD, 2) - pow(l_CD_, 2);
    
    const double discriminant = pow(A0, 2) + pow(B0, 2) - pow(C0, 2);
    if (discriminant < 0) {
        ROS_ERROR("虚根出现");
        return {0, 0, 0, 0, 0, 0};
    }
    
    const double theta_i = 2 * atan2(B0 + sqrt(discriminant), A0 + C0);

    

    const Point C = {
        B.x + l_BC_ * cos(theta_i),
        B.y + l_BC_ * sin(theta_i)
    };
    
    // 计算F点坐标
    const Point F = {
        B.x + l_BE_ * cos(theta_i) - l_EF_ * sin(theta_i),
        B.y + l_BE_ * sin(theta_i) + l_EF_ * cos(theta_i)
    };
    
    // 角速度计算
    rates_ = calculate_angular_velocity(B, C, D, theta_i);
    
    // 角加速度计算
    const double ddphi_i = calculate_angular_acceleration(B, C, D, theta_i, rates_.dphi_i);

    this->phi_[1] = theta_i - phi  ;//保存角度供ROS使用
    this->phi_[2] = atan2(C.y - D.y , C.x - D.x)  ;//保存角度供ROS使用
    
    return {
        F.x, F.y,
        -omega_ * l_AB_ * sin(phi) - rates_.dphi_i * l_BE_ * sin(theta_i) - rates_.dphi_i * l_EF_ * cos(theta_i),
        omega_ * l_AB_ * cos(phi) + rates_.dphi_i * l_BE_ * cos(theta_i) - rates_.dphi_i * l_EF_ * sin(theta_i),
        -pow(omega_, 2) * l_AB_ * cos(phi) 
            - ddphi_i * l_BE_ * sin(theta_i) 
            - pow(rates_.dphi_i, 2) * l_BE_ * cos(theta_i) 
            - ddphi_i * l_EF_ * cos(theta_i) 
            + pow(rates_.dphi_i, 2) * l_EF_ * sin(theta_i),
        -pow(omega_, 2) * l_AB_ * sin(phi) 
            + ddphi_i * l_BE_ * cos(theta_i) 
            - pow(rates_.dphi_i, 2) * l_BE_ * sin(theta_i) 
            - ddphi_i * l_EF_ * sin(theta_i) 
            - pow(rates_.dphi_i, 2) * l_EF_ * cos(theta_i) 
    };
}


    AngularRates FourBarMechanism::calculate_angular_velocity(const Point& B, const Point& C, 
                                     const Point& D, double theta_i) {
        const double dx_B = -omega_ * l_AB_ * sin(deg2rad(current_phi_));
        const double dy_B = omega_ * l_AB_ * cos(deg2rad(current_phi_));
        
        const double C_i = l_BC_ * cos(theta_i);
        const double S_i = l_BC_ * sin(theta_i);
        const double C_j = l_CD_ * cos(atan2(C.y - D.y, C.x - D.x));
        const double S_j = l_CD_ * sin(atan2(C.y - D.y, C.x - D.x));
        
        const double G1 = C_i * S_j - C_j * S_i;
        if (fabs(G1) < 1e-6) {
            throw std::runtime_error("奇异位置");
        }
        rates_.dphi_i = (C_j * (0 - dx_B) + S_j * (0 - dy_B)) / G1;
        rates_.dphi_j = (C_i * (0 - dx_B) + S_i * (0 - dy_B)) / G1; 
        return rates_;
    }

double FourBarMechanism::calculate_angular_acceleration(
    const Point& B, const Point& C, const Point& D, 
    double theta_i , double dphi_i) 
{
    const double ddx_B = -pow(omega_, 2) * l_AB_ * cos(deg2rad(current_phi_));
    const double ddy_B = -pow(omega_, 2) * l_AB_ * sin(deg2rad(current_phi_));
    
    const double theta_j = atan2(C.y - D.y, C.x - D.x);
    const double C_j = l_CD_ * cos(theta_j);
    const double S_j = l_CD_ * sin(theta_j);
    
    const double G1 = l_BC_ * l_CD_ * (cos(theta_i)*sin(theta_j) - cos(theta_j)*sin(theta_i));
    const double G2 = -ddx_B + pow(rates_.dphi_i, 2)*l_BC_*cos(theta_i) 
                    - pow(rates_.dphi_j, 2)*l_CD_*cos(theta_j); 
    const double G3 = -ddy_B + pow(rates_.dphi_i, 2)*l_BC_*sin(theta_i) 
                    - pow(rates_.dphi_j, 2)*l_CD_*sin(theta_j);  
        
        return (G2 * C_j + G3 * S_j) / G1;
    }
// 小函数选择内联
inline double FourBarMechanism::deg2rad(double deg) { return deg * M_PI / 180.0; }
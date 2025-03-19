#ifndef FOUR_BAR_MECHANISM_H
#define FOUR_BAR_MECHANISM_H
#include "vector"
struct Result {
        double x_F;
        double y_F;
        double v_Fx;
        double v_Fy;
        double a_Fx;
        double a_Fy;
    };
struct Point { double x; double y; };
struct AngularRates {
    double dphi_i;  
    double dphi_j;  
};

class FourBarMechanism
{
    public:
    FourBarMechanism(double omega, double l_AB, double l_BC,double l_CD,double l_AD,double l_BE,double l_EF);
    ~FourBarMechanism();
    std::vector<Result> calculate(double start_deg, double end_deg, double step_deg) ;
    double phi_[3];//保存倾角供ROS使用

    private:

    Result calculate_step(double phi_deg);
    AngularRates calculate_angular_velocity(const Point& B, const Point& C, 
                                     const Point& D, double theta_i) ;
    double calculate_angular_acceleration(const Point& B, const Point& C,
                                         const Point& D, double theta_i, 
                                         double dphi_i) ;
    inline double deg2rad(double deg);
    const double omega_;
    const double l_AB_, l_BC_, l_CD_, l_AD_, l_BE_, l_EF_;
    double current_phi_ = 0; 
    AngularRates rates_;

        
};
#endif
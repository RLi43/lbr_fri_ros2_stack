#include "lbr_fri_ros2/quaternion.hpp"
#include "rclcpp/logging.hpp"

namespace lbr_fri_ros2{    
    Quaternion Quaternion::slerp(Quaternion q1, Quaternion q2, double lambda)
    {
        float dotproduct = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
        float theta, st, sut, sout, coeff1;
        float coeff2 = 1.0;

        if(dotproduct < 0.0){
            // from Near
            dotproduct = -dotproduct;
            coeff2 = -1.0;
        }

        theta = (float) acos(dotproduct);
        if (theta<0.0) theta=-theta;

        if (theta < 1e-3){
            // Two quaternions are very close
            return Quaternion(q2.w, q2.x, q2.y, q2.z);
        }
        
        // algorithm adapted from Shoemake's paper
        lambda=lambda/2.0;

        st = (float) sin(theta);
        sut = (float) sin(lambda*theta);
        sout = (float) sin((1-lambda)*theta);
        coeff1 = sout/st;
        coeff2 *= sut/st;

        double qrw = coeff1*q1.w + coeff2*q2.w;
        double qrx = coeff1*q1.x + coeff2*q2.x;
        double qry = coeff1*q1.y + coeff2*q2.y;
        double qrz = coeff1*q1.z + coeff2*q2.z;

        if(std::isnan(qrw) || std::isnan(qrx) || std::isnan(qry) || std::isnan(qrz)){
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("Quaternion"), "slerp failed");
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Quaternion"), 
                "q1 " << q1.w << ", " << q1.x << ", " << q1.y << ", " << q1.z
                << "; q2 " << q2.w << ", " << q2.x << ", " << q2.y << ", " << q2.z
                );
            return Quaternion(q2.w, q2.x, q2.y, q2.z);
        }

        return Quaternion(qrw, qrx, qry, qrz);
    }

    double Quaternion::rot_angle(Quaternion q1, Quaternion q2)
    {
        // theta = arccos(2*<q1, q2>^2 - 1)
        double inner_product = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
        return acos(2*inner_product*inner_product - 1);
    }
}
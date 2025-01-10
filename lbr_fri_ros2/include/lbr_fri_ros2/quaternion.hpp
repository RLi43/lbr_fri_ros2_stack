#ifndef LBR_FRI_ROS2__QUATERNION_HPP_
#define LBR_FRI_ROS2__QUATERNION_HPP_

#include <cmath>

namespace lbr_fri_ros2{
class Quaternion{
public:
    Quaternion(): w(1.0), x(0.0), y(0.0), z(0.0){};
    Quaternion(double _w, double _x, double _y, double _z){
        w = _w;
        x = _x;
        y = _y;
        z = _z;
        normalize();
    }

    inline double norm(){
        return sqrt(w*w + x*x + y*y + z*z);
    }
    inline void normalize(){
        double n = norm();
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }
    inline void conjugate(){
        x = -x;
        y = -y;
        z = -z;
    }

    inline double get_w(){return w;}
    inline double get_x(){return x;}
    inline double get_y(){return y;}
    inline double get_z(){return z;}

    static Quaternion slerp(Quaternion q1, Quaternion q2, double lambda);
    static double rot_angle(Quaternion q1, Quaternion q2);

private:
    double w, x, y, z;
};

}

#endif // LBR_FRI_ROS2__QUATERNION_HPP_
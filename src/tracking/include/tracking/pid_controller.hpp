#ifndef TRACKING__PID_CONTROLLER_HPP
#define TRACKING__PID_CONTROLLER_HPP

#include<cmath>

namespace tracking
{
    class pidcontroller
    {
        public:
            pidcontroller(double min_ref_,double max_ref_,double min_output_,double max_output_);
            void set_pid(double new_kp, double new_ki, double new_kd);
            double get_output(double new_ref_);
        
            private:
            double min_ref, max_ref, min_output, max_output;
            double prev_error, integral_error;
            double k_p, k_i, k_d;

            
    };
}

#endif
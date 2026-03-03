#include"tracking/pid_controller.hpp"
#include<cmath>
#include<algorithm>

namespace tracking
{
    pidcontroller::pidcontroller(double min_ref_,double max_ref_,double min_output_,double max_output_)
    {
        min_ref = min_ref_;
        max_ref = max_ref_;
        min_output = min_output_;
        max_output = max_output_;

        k_p = 0.41;
        k_i = 0.06;
        k_d = 0.53;
    }

    void pidcontroller::set_pid(double new_kp, double new_ki, double new_kd)
    {
        k_p = new_kp;
        k_i = new_ki;
        k_d = new_kd;
    }

    double pidcontroller::get_output(double new_ref)
    {
        double ref = new_ref;
        double output = 0.0;

        //计算proportion
        double p_out = 0.0;
        double dist = 0.0;
        if(ref!=0)
        {
            dist = ref / std::fabs(ref);
        }

        if(std::fabs(ref)<min_ref)
        {
            p_out = 0.0;
        }
        else if(std::fabs(ref)>max_ref)
        {
            p_out = dist * max_output;
        }
        else
        {
            p_out = dist * min_output + ref * (max_output - min_output);
        }

        //计算intefral
        double i_out = (p_out + integral_error) * 2.0 / 3.0;

        //计算 derivation
        double d_out = p_out - prev_error;

        //计算output
        output = k_p * p_out + k_i * i_out + k_d * d_out;

        //更新步
        prev_error = p_out;
        integral_error = i_out;

        return std::clamp(output, -max_output, max_output);
    }
}
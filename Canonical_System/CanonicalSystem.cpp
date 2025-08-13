#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>

#include "CanonicalSystem.hpp"
#define PI 3.14f

// 构造函数实现。 dt -> delta
CanonicalSystem::CanonicalSystem(
    double x_inital_, double alpha_x_, double dt_, const char * pattern_, 
    double tau_, std::vector<double> y_demo_ )
    // 初始化 .cpp private_menber 的顺序，要跟你在 .hpp class private 的顺序一致
    : x_inital(x_inital_),
      alpha_x(alpha_x_),
      dt(dt_),
      pattern(pattern_),
      tau(tau_),
      x_last(0),
      y_demo(y_demo_){}

bool CanonicalSystem::select_pattern()
{
    // 比较字符串时，你应该使用 strcmp 函数，它会返回一个整数来指示两个字符串是否相等
    if (strcmp(pattern, "discrete") == 0) 
    {
        // 处理离散模式
        // PS： 此处 run_time = 1.0 只是默认初始值，实际情况根据 y_demo 的长度判断
        if( !y_demo.empty())
        {
            run_time = y_demo.size() * dt ;
        }
        else
        {
            run_time = 1.0 ;
        }
        std::cout << "The DMP pattern is : "<< pattern << std::endl; 
    }
    else if(strcmp(pattern, "rhythmic") == 0)
    {
        run_time = 2 * PI;
        std::cout << "The DMP pattern is : "<< pattern << std::endl;
    }
    else
    {
        std::cout << "Initialize Canonical system failed, can not recognize DMP pattern: "<< pattern << std::endl;
        return false ; 
    }
    
    return true ;
}

int CanonicalSystem::get_time_steps()
{
    // 类的成员函数可以直接访问类的私有成员变量，而不需要额外的访问权限。
    time_steps = round(run_time / dt) ;
    // tau 可以根据需要调整运动的速度或时间尺度
    time_steps = round(time_steps / tau);
    
    // std::cout << "time_steps = " << time_steps << std::endl;
    return time_steps ; 
}

std::vector<double> CanonicalSystem::generate_CS_track()
{
    std::vector<double> x_track(time_steps, 0) ;  // time_steps必须是整数类型，注意初始化类型
    x_last = x_inital ; 

    if (strcmp(pattern, "discrete") == 0) 
    {
        for(int d = 0; d < time_steps; ++d )
        {
            x_track[d] = x_last ; 
            generate_cs_discrete_track();
        }

    }
    else if(strcmp(pattern, "rhythmic") == 0)
    {
        for(int r = 0; r < time_steps; ++r )
        {
            x_track[r] = x_last ; 
            generate_cs_rhythmic_track();
        }
    }
    else
    {
        std::cout << "Initialize Canonical system failed, can not recognize DMP pattern: "<< pattern << std::endl;
    }

    // CS_track = x_track ; 

    return x_track ; 
}

void CanonicalSystem::generate_cs_discrete_track()
{
    /*
       dx/dt = - tau * alpha_x * x ==> dx = -tau * alpha_x * x * dt ; dx -> delta x
       slove: ln(x) = -tau * alpha_x * t ==> x = exp(-tau * alpha_x * t)
       so x_next = dx + x_last_time ;   
       tau float: gain on execution time increase tau to make the system execute faster
       error_coupling float: slow down if the error is > 1
    */
    double dx = - tau * alpha_x * x_last * dt ;
    x_last += dx ;  
}

void CanonicalSystem::generate_cs_rhythmic_track()
{
    x_last += tau * dt ; 
}

// return value to demo_discrete
double CanonicalSystem::get_run_time()
{
    return run_time;
}

double CanonicalSystem::get_tau()
{
    return tau ; 
}

double CanonicalSystem::get_alpha_x()
{
    return alpha_x ; 
}

double CanonicalSystem::get_dt()
{
    return dt;
}
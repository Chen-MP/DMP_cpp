#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>

#include "canonical_system.hpp"
#define PI 3.14f

// 构造函数实现。 dt_ -> delta
// 初始化 .cpp private_menber 的顺序，要跟你在 .hpp class private 的顺序一致
CanonicalSystem::CanonicalSystem(
    const double &x_inital, const double &alpha_x, const double &dt, 
    const char *pattern, const double &tau, std::vector<double> y_demo)
    : x_inital_(x_inital), alpha_x_(alpha_x), dt_(dt), pattern_(pattern),
      run_time_(0), tau_(tau), time_steps_(0), x_last_(0), y_demo_(y_demo) {}

bool CanonicalSystem::selectPattern()
{
    // 比较字符串时，你应该使用 strcmp 函数，它会返回一个整数来指示两个字符串是否相等
    if (strcmp(pattern_, "discrete") == 0) {
        // 离散模式
        // TODO: 此处 run_time_ = 1.0 只是默认初始值，实际情况根据 y_demo_ 的长度判断
        if ( !y_demo_.empty() ) {
            run_time_ = y_demo_.size() * dt_ ;
        } else {
            run_time_ = 1.0 ;
        }
        std::cout << "The DMP pattern_ is : "<< pattern_ << std::endl; 
    
    } else if (strcmp(pattern_, "rhythmic") == 0) {
        run_time_ = 2 * PI;
        std::cout << "The DMP pattern_ is : "<< pattern_ << std::endl;
    
    } else {
        std::cout << "Initialize Canonical system failed, can not recognize DMP pattern_: "<< pattern_ << std::endl;
        return false ; 
    }
    
    return true ;
}

int CanonicalSystem::getTimeSteps()
{
    // 类的成员函数可以直接访问类的私有成员变量，而不需要额外的访问权限。
    time_steps_ = round(run_time_ / dt_);
    // tau_ 可以根据需要调整运动的速度或时间尺度
    time_steps_ = round(time_steps_ / tau_);
    
    return time_steps_ ; 
}

std::vector<double> CanonicalSystem::generateCanonicalSystemTrack()
{
    std::vector<double> x_track(time_steps_, 0); // time_steps必须是整数类型，注意初始化类型
    x_last_ = x_inital_ ; 

    if (strcmp(pattern_, "discrete") == 0) {
        for (int i = 0; i < time_steps_; ++i) {
            x_track[i] = x_last_ ; 
            generateCanonicalSystemDiscreteTrack();
        }

    } else if (strcmp(pattern_, "rhythmic") == 0) {
        for (int j = 0; j < time_steps_; ++j) {
            x_track[j] = x_last_ ; 
            generateCanonicalSystemRhythmicTrack();
        }

    } else {
        std::cout << "Initialize Canonical system failed, can not recognize DMP pattern_: "<< pattern_ << std::endl;
    }

    // CS_track = x_track ; 

    return x_track ; 
}

// 规范系统
void CanonicalSystem::generateCanonicalSystemDiscreteTrack()
{
    /*
       dx/dt_ = - tau * alpha_x * x(t) ==> dx = -tau * alpha_x * x(t) * dt; dx -> delta_x
       slove: ln[x(t)] = -tau * alpha_x * t ==> x = exp(-tau * alpha_x * t)
       Discretization  x_next = dx + x_last_time ;   
       tau_ float: gain on execution time increase tau_ to make the system execute faster
       error_coupling float: slow down if the error is > 1
    */
    double dx = - tau_ * alpha_x_ * x_last_ * dt_ ;
    x_last_ += dx ;  
}

void CanonicalSystem::generateCanonicalSystemRhythmicTrack()
{
    x_last_ += tau_ * dt_ ; 
}

// return value to demo_discrete
double CanonicalSystem::getRunTime()
{
    return run_time_;
}

double CanonicalSystem::getTau()
{
    return tau_ ; 
}

double CanonicalSystem::get_alpha_x()
{
    return alpha_x_ ; 
}

double CanonicalSystem::get_dt()
{
    return dt_;
}
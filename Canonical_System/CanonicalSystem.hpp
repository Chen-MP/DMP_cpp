#pragma once

#include <vector>

// 声明
class CanonicalSystem
{
private:
    /* data */
    double x_inital ;
    double alpha_x ; 
    double dt ; 
    const char * pattern ;
    double run_time ; 
    double tau ; 
    int time_steps ; 
    double x_last ; // next x
    std::vector<double> y_demo ;

    // 缓存的数据 cached data
    // std::vector<double> CS_track;

public:
    // 使用const char *作为函数参数类型可以保证安全性和灵活性，并且在处理字符串时是一种常见做法
    // KEY : 构造函数的名称应该与类名相同。构造函数没有返回类型，因此不能像普通函数那样指定返回类型。
    CanonicalSystem(
        double x_inital_, double alpha_x_, double dt_, 
        const char * pattern_, double tau_, std::vector<double> y_demo_ 
        );
    virtual ~CanonicalSystem() = default ;
     
    /*
    std::vector<double> 是 C++ 标准库中的一个容器类模板，用于表示动态数组。
    std::vector<double> 表示一个元素类型为 double 的动态数组，可以动态地增长和缩小。
    你可以使用 std::vector<double> 来替代普通数组，在需要动态管理大小的情况下特别有用。
    */
    bool select_pattern() ;
    std::vector<double> generate_CS_track() ;
    void generate_cs_discrete_track();
    void generate_cs_rhythmic_track();

    // return value to demo_discrete
    double get_run_time();
    int get_time_steps() ; 
    double get_tau() ; 
    double get_alpha_x();
    double get_dt();
};



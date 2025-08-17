#pragma once

#include <vector>

// 声明
class CanonicalSystem {
public:
    // 使用const char *作为函数参数类型可以保证安全性和灵活性，并且在处理字符串时是一种常见做法
    // KEY : 构造函数的名称应该与类名相同。构造函数没有返回类型，因此不能像普通函数那样指定返回类型。
    CanonicalSystem(
        const double &x_inital, const double &alpha_x, const double &dt, 
        const char *pattern, const double &tau, std::vector<double> y_demo);
    virtual ~CanonicalSystem() = default ;
     
    /*
    std::vector<double> 是 C++ 标准库中的一个容器类模板，用于表示动态数组。
    std::vector<double> 表示一个元素类型为 double 的动态数组，可以动态地增长和缩小。
    你可以使用 std::vector<double> 来替代普通数组，在需要动态管理大小的情况下特别有用。
    */
    bool selectPattern() ;
    std::vector<double> generateCanonicalSystemTrack() ;
    void generateCanonicalSystemDiscreteTrack();
    void generateCanonicalSystemRhythmicTrack();

    // return value to demo_discrete
    double getRunTime();
    int getTimeSteps() ; 
    double getTau() ; 
    double get_alpha_x();
    double get_dt();

private:
    /* data */
    double x_inital_ ;
    double alpha_x_ ;  // 控制收敛的快慢
    double dt_ ; 
    const char * pattern_ ;
    double run_time_ ; 
    double tau_ ; // 拉伸时钟长度
    int time_steps_ ; 
    double x_last_ ; // next x
    std::vector<double> y_demo_ ;

    // 缓存的数据 cached data
    // std::vector<double> CS_track;
};



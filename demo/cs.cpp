#include <Python.h>
#include <vector>
#include <string>
#include "canonical_system/canonical_system.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
    double x0 = 1.0;
    double alpha_x = 1.0;
    double dt = 0.01 ;
    const char* pattern = "discrete" ;
    // const char* pattern = "A" ;
    double tau = 1.0; 
    
    int length_y_demo = 500 ;  
    std::vector<double> y_demo(length_y_demo, 0);
    for ( int i = 0; i < length_y_demo; ++i) {
        y_demo[i] = sin(i * dt) ; 
    } 
    /*
       std::vector<double> y_demo;
       for ( int i = 0; i < 500; ++i)
       {
           y_demo[i] = sin(i * dt) ; 
       } 
      在这段代码中，存在一个问题。在使用 std::vector 的情况下，
      您应该使用 push_back 函数向向量中添加元素，而不是直接通过索引访问元素。
      因为在初始化时，std::vector 并没有分配足够的空间来存储 500 个元素，
      所以直接通过索引访问会导致越界访问，造成未定义行为。

      y_demo.push_back(sin(i * dt));
    */

    // std::cout << "y_demo.size()" <<y_demo.size() << std::endl ;

    // {} 表示 y_demo 输入的时空数组
    CanonicalSystem cs(x0, alpha_x, dt, pattern, tau, {});
    if (!cs.selectPattern()) {
        return 0 ;
    }
    int time_steps = cs.getTimeSteps();

    std::vector<double> x(time_steps), y(time_steps) ; 
    y = cs.generateCanonicalSystemTrack();

    for (int num_steps = 0; num_steps < time_steps; ++ num_steps) {
        x.at(num_steps) = num_steps ; 
    }

    // canonical_system cs1(x0, alpha_x*0.1, dt, pattern, tau*0.5, time_steps);
    // cs1.selectPattern();
    // time_steps = cs1.getTimeSteps();

    // std::vector<double> x1(time_steps), y1(time_steps) ; 
    // y1 = cs1.run();
    // for (int num_steps_01 = 0; num_steps_01 < time_steps; ++ num_steps_01)
    // {
    //     x1.at(num_steps_01) = num_steps_01 ; 
    // }

    /*
    1.第一种写法
    设置要显示的文字
    std::vector<std::string> labels = {"Line 1", "Line 2"};
    std::map<std::string, std::string> lin1_01;
    lin1_01.insert({"label", "system trajectory"});
    lin1_01.insert({"color", "r"});
    lin1_01.insert({"ls", "scatter"});
    
    plt::plot(x, y, lin1_01);

    2.
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("label", "system trajectory"));
    */

    // 一个图多条线
    // plt::plot(x, y, "r-",x1, y1, "b-");
    plt::plot(x, y,{{"color","b"},{"label","line01"},{"ls","-"}});
    // plt::plot(x1, y1,{{"color","r"},{"label","line01"},{"ls","-"}});
    // plt::scatter(x1,y1);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::title("Plot Title");
    plt::legend();

    // 多个图一条线
    // plt::figure();
    // plt::subplot(2, 1, 1);
    // plt::plot(x, y, "r-");  // Plot first line in first subplot

    // plt::subplot(2, 1, 2);
    // plt::plot(x1, y1, "b-");  // Plot second line in second subplot

    plt::show();

    return 0 ;
}
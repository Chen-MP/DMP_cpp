#include <Python.h>
#include <vector>
#include <string>
#include <cmath>

#include "original_discrete/original_discrete.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


int main()
{

    // std::wstring pythonHome = L"F:\\Python38";
    // Py_SetPythonHome(L"/usr");
    // Py_SetPythonHome(pythonHome.c_str());

    double x0 = 1.0 ;
    // 要执行浮点数除法，应该将其中一个操作数转换为浮点数
    double alpha_x = 1 ;
    float dt = 0.01f ;
    const char* pattern = "discrete" ;
    // const char* pattern = "A" ;
    double tau = 1 ; 
    int num_gbfs = 10 ; 

    int length_y_demo = 500 ; 
    std::vector<double> y_demo(length_y_demo, 0) ;
    for(int len = 0; len < length_y_demo; ++len)
    { 
        y_demo[len] = sin(len * 0.01);
    }
    double goal = y_demo[ length_y_demo - 1 ] ; 

    std::vector<std::vector<double>> Psi(num_gbfs);
    //   std::unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(
    //   matrix_m1, matrix_m2, matrix_inequality_constrain,
    //   matrix_inequality_boundary, matrix_equality_constrain,
    //   matrix_equality_boundary));
    OriginalDiscrete dis( x0, alpha_x, 0, dt, pattern, tau,
                           num_gbfs, y_demo, goal,{});

    if (!dis.selectPattern())
    {
        return 0 ;
    }

    int time_steps = dis.getTimeSteps();
    // std::cout << "error " << std::endl ;
    std::vector<double> x(time_steps) ; 
    // std::vector<double> y2(time_steps) ; 
    std::vector<std::vector<double>> y(num_gbfs); 
    y = dis.generate_gbfs_Psi();
    
    for (int num_steps = 0; num_steps < time_steps; ++ num_steps)
    {
        x.at(num_steps) = num_steps ; 
    }
    
    for(int k =0 ; k<num_gbfs;++k)
    {
        plt::plot(x,y[k]);
    }
    // y2 = dis.generateCanonicalSystemTrack();
    // plt::plot(x,y2);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::title("Plot Title");
    plt::legend();

    plt::show();

                                  
}
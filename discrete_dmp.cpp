#include <Python.h>
#include <vector>
#include <string>
#include <chrono>

#include "Original_discrete/original_discrete.hpp"
// #include "CanonicalSystem.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


int main()
{
    auto start = std::chrono::steady_clock::now(); // 计时

    std::wstring pythonHome = L"F:\\Python38";
    Py_SetPythonHome(pythonHome.c_str());

    double x0 = 1 ;  // Canonical_System 的初始值，最终收敛到 0
    int alpha_y = 25 ; 
    double alpha_x =  1 ;
    float dt = 0.01f ;
    const char* pattern = "discrete" ;
    // const char* pattern = "A" ;
    double tau = 1 ; 
    int num_gbfs = 200 ; 
    
    // y demostrate = y desired
    int length_y_demo = 500 ; 
    std::vector<double> y_demo(length_y_demo, 0) ;
    std::vector<double> obstacle= {} ; // 空数组
    for(int len = 0; len < length_y_demo; ++len)
    {
        // y_demo[len] = 1;
        // if(20 <= len && len <= 59)
        // {
        //     y_demo[len] = 0.5;
        // }
        // if( 60 <= len && len <= length_y_demo)
        // {
        //     y_demo[len] = 0;
        // }
        y_demo[len] = sin(len * dt);
    }
    
    double goal = y_demo[ length_y_demo - 1 ] ; 


    std::vector<std::vector<double>> Psi(num_gbfs);
    //   std::unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(
    //   matrix_m1, matrix_m2, matrix_inequality_constrain,
    //   matrix_inequality_boundary, matrix_equality_constrain,
    //   matrix_equality_boundary));
    original_discrete dis( x0, alpha_x, alpha_y , dt, pattern, tau,
                           num_gbfs, y_demo, goal, obstacle);

    if (!dis.select_pattern())
    {
        return 0 ;
    }

    int time_steps = dis.get_time_steps();
    // std::cout << "error " << std::endl ;
    std::vector<double> x(time_steps) ; 
    std::vector<double> y_dmp(time_steps) ; 
    // std::vector<double> dot_y(time_steps) ;
    // std::vector<double> ddot_y(time_steps) ;
    y_dmp = dis.generate_DMP_track();
    for (int num_steps = 0; num_steps < time_steps; ++ num_steps)
    {
        x.at(num_steps) = num_steps * dt ; 
    }
    // dot_y = dis.generate_derivation(y_demo);
    // ddot_y = dis.generate_derivation(dot_y);

    std::vector<double> y2(time_steps) ; 
    y2 = dis.generate_CS_track();

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " milliseconds" << std::endl; // 结束计时

    plt::figure(1) ; 
    plt::plot(x,y2);
    
    plt::figure(2) ; 
    plt::plot(x, y_demo,{{"label","y_demo"}}) ; 
    plt::plot(x, y_dmp, {{"label","y_dmp"}});
    // plt::plot(x, dot_y);
    // plt::plot(x, ddot_y);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::title("Plot Title");
    plt::legend();

    plt::show();

                                  
}
#pragma once 

#include <vector>

// #include "CanonicalSystem.hpp"
#include "Canonical_System/CanonicalSystem.hpp"

class original_discrete : public CanonicalSystem
{
private:
    /* data */
    // int num_dmps ; // number of data dimensions, one dmp for one degree
    int num_gbfs ;  // number of Gaussian basis functions
    double alpha_y ; 
    double beta_y ; 
    std::vector<double> y_demo;
    double y_inital ; 
    double goal;
    std::vector<double> obstacle ;

public:
    original_discrete(double x_inital_, double alpha_x_, double alpha_y_, double dt_, const char * pattern_, double tau_,
                      int num_gbfs_, std::vector<double> y_demo_, double goal_, std::vector<double> obstacle_);
    virtual ~original_discrete() = default ;
    
    std::vector<std::vector<double>> generate_gbfs_Psi();
    std::vector<double> generate_derivation(std::vector<double> path);
    std::vector<double> generate_f_target();
    double generate_scaling_term() ;
    std::vector<double> generate_weights();
    std::vector<double> generate_force_term();
    std::vector<double> generate_DMP_track() ; 

};


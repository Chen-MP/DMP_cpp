#pragma once 

#include <vector>

// #include "canonical_system.hpp"
#include "canonical_system/canonical_system.hpp"

class OriginalDiscrete : public CanonicalSystem {
private:
    /* data */
    // int num_dmps ; // number of data dimensions, one dmp for one degree
    int num_gbfs_;    // number of Gaussian basis functions
    double alpha_y_ ; 
    double beta_y_ ; 
    std::vector<double> y_demo_;
    double y_inital_ ; 
    double goal_;
    std::vector<double> obstacle_ ;

public:
    OriginalDiscrete(
        const double &x_inital, const double &alpha_x, const double &alpha_y, 
        const double &dt, const char *pattern, const double &tau, 
        const int &num_gbfs, std::vector<double> y_demo, double goal, 
        std::vector<double> obstacle);
    virtual ~OriginalDiscrete() = default ;
    
    std::vector<std::vector<double>> generate_gbfs_Psi();
    std::vector<double> generate_derivation(std::vector<double> path);
    std::vector<double> generate_f_target();
    double generate_scaling_term() ;
    std::vector<double> generate_weights();
    std::vector<double> generate_force_term();
    std::vector<double> generate_DMP_track() ; 

};


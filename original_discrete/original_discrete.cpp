#include <cmath>
#include <iostream>

#include "original_discrete.hpp"


OriginalDiscrete::OriginalDiscrete
    ( const double &x_inital, const double &alpha_x, const double &alpha_y, 
      const double &dt, const char *pattern, const double &tau, 
      const int &num_gbfs, std::vector<double> y_demo, double goal, 
      std::vector<double> obstacle )
        : CanonicalSystem(x_inital, alpha_x, dt, pattern, tau, y_demo), 
          num_gbfs_(num_gbfs), alpha_y_(alpha_y), beta_y_(alpha_y / 4),
          y_demo_(y_demo), y_inital_(y_demo[0]), goal_(goal),
          obstacle_(obstacle) {}

/*
   Guassian basis functions:
   popular :f(x)= 1/[(2π)^0.5 * σ] * exp[-(x-μ)^2/(2*σ^2)] 
    --> paper : Psi = exp[-(x-μ)^2/(2*σ^2)]  
    --> blog :  Psi = exp[- hi * (x-μ)^2]   hi is variance 方差 
   在高斯函数中，μ 决定了函数的中心位置，即函数在 x=μ 处取得最大值；
   而 σ 决定了函数的宽度，即标准差越大，函数图像越矮胖，标准差越小，函数图像越高瘦。

   CaninicalSystem 的收敛速度是越来越慢的，那么如果我们把每个基函数的宽度在的维度上设定为一样的，
   就会导致早期的基函数持续时间短，而后期的基函数持续时间长。因为时间越往后，经过同样宽度的所需的时间越多，
   从而导致在时间的维度上出现基函数的宽度不一致的问题，时间约靠后的宽度越大。因此，基函数的宽度应time_steps该随着时间的递增而递减，从而保证在时间的维度上，每个基函数的宽度也是均匀的。
   the variance of basis function i is equal to the number of basis functions divided by the center of that basis function.
   So hi = num_gbfs_ /  gbfs_center_point[d]
*/
std::vector<std::vector<double>> OriginalDiscrete::generate_gbfs_Psi()
{
    // Set the centre of the Gaussian basis functions be spaced evenly throughout run time
    // 生成基础高斯函数的中心点, 根据 run_time 均匀分布
    // std::vector<double> gbfs_center_point(num_gbfs_) ; 
    std::vector<double> gbfs_variance(num_gbfs_) ;
    
    std::vector<double> discrete_x_track = generateCanonicalSystemTrack() ; // return x_track , number : time_steps
    // getTimeSteps() = run().size() = run_track.size
    int discrete_time_steps = getTimeSteps() ;  
    // 构造容器，装载数组
    std::vector<std::vector<double>> discrete_Psi(num_gbfs_);
    std::vector<double> t_center(num_gbfs_);
    std::vector<double> t_track(discrete_time_steps);
    std::vector<double> Psi_center(num_gbfs_) ; 
    double distance = getRunTime() / num_gbfs_ ; // 时间上分布，将run_time上的均匀分布

    /* first method : come from 沙漏 */
    // for(int h = 0; h < num_gbfs_; ++h) {
    //     t_center[h] = distance * h ; 
    //     for(int t = 0; t < discrete_time_steps; ++t) {
    //         t_track[t] = t * delta_t;
    //         // abs() -> int ; std::abs -> double
    //         if (std::abs(t_center[h]-t_track[t]) <= delta_t) {
    //             Psi_center[h] = discrete_x_track[t] ; 
    //         }
    //     }
    // }
    
    /* second method : exp */
    // dis_gbfs_center -> The distance between the guassian basis function center points 
    // it is not time_steps / num_gbfs_
    double tau = getTau();
    double alpha_x = get_alpha_x();

    for (int d = 0; d < num_gbfs_; ++d ) {
        // 创建容器，长度为 discrete_time_steps，每个元素都初始化为 0
        std::vector<double> Psi_track(discrete_time_steps, 0) ;
        
        // x = exp(-tau * alpha_x * t)
        // 将run_time上的均匀分布，映射到 Canonical System 对应的 x 值上
        // gbfs_center_point[d] =  exp(- tau * alpha_x * Psi_center[d]) ;
        Psi_center[d] = exp( -tau * alpha_x * distance * d) ;

        gbfs_variance[d] = (std::pow(num_gbfs_, 1.5) / Psi_center[d]) / alpha_x ;
        
        for (int k = 0; k < discrete_time_steps; ++k) {   
            // std::pow() 平方
            Psi_track[k] = exp( -1 * gbfs_variance[d] *  std::pow((discrete_x_track[k] - Psi_center[d]), 2));
            
            if (Psi_track[k] < 1e-5) {
                Psi_track[k] = 0 ;
            }
        }

        discrete_Psi[d] = Psi_track ; // number : time_steps
    }
    
    return discrete_Psi ;
}


std::vector<double> OriginalDiscrete::generate_derivation(std::vector<double> path)
{
    // 保证path的数组长度是 time_steps; 即 path.size() = time_steps
    int size = path.size();
    double delta_t = get_dt();
    std::vector<double> dot_path(size) ;

    // Compute derivative using central difference method
    for (int length = 1; length < (size - 1); ++length) {
        dot_path[length] = (path[length+1] - path[length-1]) / (2*delta_t) ; 
    }
    // Special cases for endpoints
    dot_path[0] = (path[1] - path[0]) / delta_t;  // Forward difference at the beginning
    dot_path[size-1] = (path[size - 1] - path[size - 2]) / delta_t;  // Backward difference at the end

    return dot_path;
}


std::vector<double> OriginalDiscrete::generate_f_target()
{
    double tau = getTau() ;
    int time_steps = getTimeSteps() ;
    
    std::vector<double> dot_y_demo(time_steps, 0) ; 
    std::vector<double> ddot_y_demo(time_steps, 0) ;
    std::vector<double> f_target(time_steps, 0) ;

    dot_y_demo = generate_derivation(y_demo_) ; 
    ddot_y_demo = generate_derivation(dot_y_demo) ; 

    for (int g = 0; g < time_steps; ++g) {
        f_target[g] = tau * tau * ddot_y_demo[g] 
                      - alpha_y_ * (beta_y_ * (goal_ - y_demo_[g]) 
                      - tau * dot_y_demo[g]);
    }
    
    return f_target;
}


// 空间缩放项
double OriginalDiscrete::generate_scaling_term()
{
    double scaling_term = goal_ - y_inital_ ; 
    return scaling_term ;
}


/*
   Generate a set of weights over the basis functions such that the target forcing term trajectory is matched.
   efficiently calculate BF weights using weighted linear regression
   ->使用加权线性回归有效计算BF权重
   time_steps --> t
   s : t x 1 ; gamma t x t ; f_target
    wi = (s.T * gamma * f_target) / (s.T * gamma * s)
    分子：（1 x t ） * (t x t ) * (t x 1) ==> 1 x 1
    s = 缩放项 ； gamma = Psi ;  
*/
std::vector<double> OriginalDiscrete::generate_weights()
{
    std::vector<std::vector<double>> Psi = generate_gbfs_Psi();
    std::vector<double> cs_track = generateCanonicalSystemTrack() ;
    int t = getTimeSteps() ; 

    std::vector<double> f_target = generate_f_target();
    std::vector<double> wi(num_gbfs_); // 与高斯数量有关
    double scaling_item = generate_scaling_term();

    for (int m = 0; m < num_gbfs_; ++m) {
        // 重置
        std::vector<double> molecule(num_gbfs_, 0);  // 分子
        std::vector<double> denominator(num_gbfs_, 0);  // 分母

        for (int n = 0; n < t; ++n) {
            molecule[m] += cs_track[n] * Psi[m][n] * f_target[n];
            // scaling_item[n] = goal_ - y_demo_[n] ; 
            // 寄寄寄！！！！这里犯病了，忘了加abs()，还要加 std::
            if (std::abs(scaling_item) > 1e-5) {
                denominator[m] += std::pow(cs_track[n], 2) * Psi[m][n] * scaling_item ; 
            } else {
                denominator[m] += std::pow(cs_track[n], 2) * Psi[m][n]  ;
            }
        }
        wi[m] = molecule[m] / denominator[m] ;
        // 这行代码的作用是将 wi 中的 NaN（Not a Number）值替换为 0。
        // wi 包含 NaN 值，那么这行代码将把所有的 NaN 值替换为 0，并将结果存储回 self.w 中。
        if (std::isnan(wi[m])) {
            wi[m] = 0.0;
        }
    }

    return wi ; 
}


std::vector<double> OriginalDiscrete::generate_force_term()
{
    int time = getTimeSteps() ;
    std::vector<double> force_term(time,0) ; 
    std::vector<std::vector<double>> Psi = generate_gbfs_Psi() ;
    std::vector<double> wi = generate_weights() ; 
    std::vector<double> cs_track = generateCanonicalSystemTrack() ;
    double scaling_term = generate_scaling_term() ;

    std::vector<double> mole(time,0) ;  // 分子
    std::vector<double> deno(time,0) ;  // 分母
    /*
       force_term = sum(psi * wi)/ sum(psi) * cs_track * scaling_term
       tx1 = [(t x 1) * (1 x 1)] / (1 x 1) * (t x 1) * (1 x 1) 
    */
    for (int i = 0; i < time; ++i) {
        for (int j = 0; j < num_gbfs_; ++j) {   
            /*
              PS: 这里要理清楚逻辑
              大循环是time，小循环是num_gbfs
              注解：  
                ①分子：将 i 时刻的上（全部） num_gbfsg 个数的 Psi_track 点乘以 各自的 weight， 并且计算累加和
                ②分母：将 i 时刻的上（全部） num_gbfsg 个数的 Psi_track 点累加
            */
            mole[i] += Psi[j][i] * wi[j]  ; // t x 1
            deno[i] += Psi[j][i] ;  // t x 1
        }
    }

    for (int k = 0; k < time; k++) {
        // 防止分母为 0
        if (deno[k] > 1e-5) {
            force_term[k] = (mole[k] * cs_track[k] * scaling_term ) / deno[k]  ;
        }
    }
    return force_term ; 

}


std::vector<double> OriginalDiscrete::generate_DMP_track()
{
    int time = getTimeSteps() ;
    std::vector<double> ddot_y_dmp(time, 0);
    std::vector<double> dot_y_dmp(time, 0) ; 
    std::vector<double> y_dmp(time, 0) ;
    y_dmp[0] = y_inital_ ;
    std::vector<double> force = generate_force_term();
    double delta_t = get_dt();
    double tau = getTau();
    double p = 0.0 ; 
    
    double ref = std::abs(generate_scaling_term()) * 0.3   ;
    // KEY:
    // TODO: 这里十分关键，Py代码初始化太粗糙了，搞得我都搞不清别人输入的类型
    //       一定要注意 相加 的 逻辑 
    for (int u = 0; u < time ; ++u) {

        if( u == 0 ) {
            ddot_y_dmp[u] = alpha_y_ * (beta_y_ * (goal_ - y_dmp[u]) - dot_y_dmp[u]) 
                            + force[u] ; 
            dot_y_dmp[u] += ddot_y_dmp[u] * tau * delta_t ;
            y_dmp[u] += dot_y_dmp[u] * tau * delta_t ;
        } else {
            if( obstacle_.empty() ) {
                // 空数组 {} 即是 没有障碍物
                ddot_y_dmp[u] = alpha_y_ * (beta_y_ * (goal_ - y_dmp[u-1]) - dot_y_dmp[u-1]) 
                                + force[u] ;
                // std::cout << " empty " << std::endl;
            } else {
                double gamma = 100.0 ; 
                double pi = M_PI;
                double beta = 20.0 / pi ; 
                double x = u * delta_t ; 
                // distance_world = obstacle_world - dmp_world
                double distance_x = obstacle_[0] - x ; 
                double distance_y = obstacle_[1] - y_dmp[u-1] ;
                // 障碍物 与 DMP 轨迹 的 欧几里得距离
                double difference = sqrt(std::pow(distance_x, 2) + std::pow(distance_y, 2)); 
                
                std::vector<double> R_halfpi = { cos( pi / 2.0 ), - sin( pi / 2.0 ),
                                                 sin( pi / 2.0 ),   cos( pi / 2.0 ) } ; 
                
                if (difference < ref) {
                    // -atan2(...) = 把坐标系旋转，使当前速度方向对齐 x 轴，方便后续计算障碍物相对角度
                    // 相当于 向量 顺时针旋转，为负
                    // 或许理解为 添加负号，就是 世界坐标系 相对于 局部坐标系 的旋转角度
                    //          添加正号，就是 局部坐标系 相对于 世界坐标系 的旋转角度
                    // theta^{local}_{world}
                    double theta_dy = -std::atan2(dot_y_dmp[u-1], x);
                    // 世界坐标系 相对于 局部坐标系 的 旋转矩阵 R^{local}_{world}
                    std::vector<double> rotation = { std::cos(theta_dy), -std::sin(theta_dy),
                                                     std::sin(theta_dy),  std::cos(theta_dy) } ;
                    // 将 障碍物坐标 转换到 局部坐标系下 R^{local}_{world} * distance^{world}
                    std::vector<double> obstacle_local = { rotation[0] * distance_x + rotation[1] * distance_y,
                                                           rotation[2] * distance_x + rotation[3] * distance_y };
                    // 障碍物 相对 局部坐标系x轴下 的偏角
                    double theta_obs = std::atan2(obstacle_local[1], obstacle_local[0]);
                    // dphi 表示“角速度偏移”，也就是障碍物引起的转向量；幅度；权重；大小
                    double dphi = gamma * theta_obs * exp( -beta * std::abs(theta_obs) );
                    // (obstacle_world - dmp_world) x dot_dmp 外积 不是严格的叉乘
                    // 将 距离信息 和 速度信息 结合起来，生成一个 加权方向矩阵，表明障碍物对当前速度方向的影响
                    // 求外积 a = [a1, a2]，b = [b1, b2] 2x1  ; np.outer(obstacle_ - y, dy)
                    //     a1*b1 a1*b2
                    //     a2*b1 a2*b2
                    std::vector<double> outer = { distance_x * x, distance_x * dot_y_dmp[u-1],
                                                  distance_y * x, distance_y * dot_y_dmp[u-1] };
                    // 旋转 90°，得到沿速度方向正交的修正方向；方向
                    std::vector<double> R = {R_halfpi[0] * outer[0] + R_halfpi[1] * outer[2], R_halfpi[0] * outer[1] + R_halfpi[1] * outer[3],
                                             R_halfpi[2] * outer[0] + R_halfpi[3] * outer[2], R_halfpi[2] * outer[1] + R_halfpi[3] * outer[3]};
                    // 乘以权重 dphi → 得到最终加到 DMP 的避障加速度
                    // 式子 (3.2)
                    std::vector<double> pval = { -(R[0] * x + R[1] * dot_y_dmp[u-1]) * dphi,
                                                 -(R[2] * x + R[3] * dot_y_dmp[u-1]) * dphi };
                    p += pval[1];
                    ddot_y_dmp[u] = alpha_y_* (beta_y_ * (goal_ - y_dmp[u-1]) - dot_y_dmp[u-1]) 
                                    + force[u] + p ;
                
                } else {
                    ddot_y_dmp[u] = alpha_y_* (beta_y_ * (goal_ - y_dmp[u-1]) - dot_y_dmp[u-1]) 
                                    + force[u] ;
                }
            }

            // ddot_y_dmp[u] = alpha_y_*(beta_y_ * (goal_ - y_dmp[u-1]) - dot_y_dmp[u-1]) + force[u] ;
            // 叠加过去生成的 数据 ； dot_y/dt += tau * ddot_y/ddot_dt => dot_y += tau* ddot_y/ddot_dt *dt 
            dot_y_dmp[u] = ddot_y_dmp[u] * tau * delta_t + dot_y_dmp[u-1];
            // y += tau * dot_y/dt 
            y_dmp[u] = dot_y_dmp[u] * tau * delta_t + y_dmp[u-1];
        }    
    }

    return y_dmp ; 
}
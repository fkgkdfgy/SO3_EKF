/**
 * 
 * 进行EKF的代码测试
 * 
*/


#include "simulator.h"
#include "so3_ekf.h"

class TestHandler
{
    public:

    void TestPropagation()
    {
        // EKF 参数设置
        Simulator sim;
        sim.params.imu_gps_interval = 2000;
        ESKFOptions options;
        options.n_a = sim.params.acc_noise_sigma * sim.params.acc_noise_sigma;
        options.n_w = sim.params.gyro_noise_sigma * sim.params.gyro_noise_sigma; 
        options.n_ba = sim.params.acc_bias_sigma * sim.params.acc_bias_sigma; 
        options.n_bw = sim.params.gyro_bias_sigma * sim.params.gyro_bias_sigma; 

        EKF ekf(options);
        auto data_pool = sim.GenerateTestData();

        // 对 EKF 进行一次初始化
        ekf.SetPosition(sim.init_pos);
        ekf.SetRotation(sim.init_rot);
        ekf.SetVelocity(sim.init_vec);
        

        ekf.RunOnce(data_pool.front());   
    }

    void TestFusion();
};



int main()
{
    TestHandler th;
    th.TestPropagation();

}
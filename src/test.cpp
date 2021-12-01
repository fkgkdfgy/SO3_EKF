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
        sim.params.imu_gps_interval = 200;
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
        
        // const auto & unit = data_pool.front();
        // ekf.RunOnce(unit);

        for(const auto & unit:data_pool)
        ekf.RunOnce(unit);
    }

    void TestData()
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

        // 检测   Acc第一个数据
        auto imu = data_pool.front().imu_pool.front();
        std::cout<<"Acc_in_b:"<<imu.Acc.transpose()<<std::endl;
        std::cout<<"Acc_in_w:"<<(Sophus::SO3d::exp(sim.init_rot)*imu.Acc).transpose()<<std::endl;

        


    }

    void TestFusion();
};



int main()
{
    TestHandler th;
    th.TestPropagation();

}
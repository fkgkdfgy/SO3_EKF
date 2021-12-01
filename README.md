# SO3_EKF
## 这个仓库存在的原因
- 很难找到使用 SO3 的Extend Kalman Filter 的实现,大多数基于四元数。
- 有的EKF实现，并不适合原理的学习。为了稳定性的考虑，有很多其他的内容
- 试着去回答一直困扰我自己的问题，SO3的EKF在进行更新的时候，旋转应该选择左乘更新还是右乘更新。
- 说明“优化框架” 和 EKF 之间的关系。推导 EKF 中的 K 和 Hessian 矩阵的关系。 EKF 中的方差更新和 Hessian 矩阵的关系

目前看来，以上的目的都达到了

## 编译方法

mkdir build
cd build
cmake .. 
make 

## 运行
./test_case

## 致谢
HeyiJia vio_simulation

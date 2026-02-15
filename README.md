# AtomImuCan

## 1. 模块作用
AtomImu CAN 通信模块。将 IMU 的 CAN 数据帧解析为姿态与角速度数据，供控制模块直接使用。
Manifest 描述：AtomImu Communication(can)

## 2. 主要函数说明
1. ThreadFunction: 周期发布欧拉角、角速度与加速度 Topic。
2. Decode: 解析收到的 CAN 帧并更新内部反馈。
3. CalcAbsAccl / CalcEulr: 计算绝对加速度和欧拉角。
4. RxCallback / CheckOffline: CAN 回调接收与离线检测。

## 3. 接入步骤
1. 添加模块并设置 CAN 参数（can_id、can_bus_name）。
2. 让下游模块订阅本模块发布的姿态 Topic。
3. 上电后先确认姿态数据连续，再进行系统联调。

标准命令流程：
    xrobot_add_mod AtomImuCan --instance-id atomimucan
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: AtomImuCan
entry_header: Modules/AtomImuCan/AtomImuCan.hpp
constructor_args:
 - param:
     can_id: 12
     can_bus_name: can1
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
  - can

Depends:
[]

## 6. 代码入口
Modules/AtomImuCan/AtomImuCan.hpp

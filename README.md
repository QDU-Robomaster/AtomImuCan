# AtomImuCan

AtomImu CAN 通信模块：接收 AtomImu 通过 CAN 发送的加速度与角速度帧，在本地解算姿态，
并发布欧拉角、去重力加速度和角速度 Topic，供控制模块使用。

## 行为

- 构造时在 `can_bus` 上注册标准帧 ID 范围 `[can_id, can_id + 4]` 的接收回调。帧类型按
  `pack.id - can_id` 区分：
  - `0`：加速度，三个 21 位字段，按 ±24 g 解码。
  - `1`：角速度，三个 21 位字段，按 ±2000 °/s 解码为 rad/s。
  - `3`（欧拉角）、`4`（四元数）：只有把头文件中的 `USE_ATOMIMU_EULER` /
    `USE_ATOMIMU_QUATERNION` 改为 1 时才解析；默认均为 0，这两类帧被忽略。
  - 其他偏移被忽略。
- 线程 `AtomImuCan`（HIGH 优先级，栈 1024）每 2 ms 运行一次：
  1. 用上一次的四元数从加速度中减去重力方向，得到去重力加速度；
  2. 默认用 Madgwick 风格的互补算法（`BETA_IMU = 0.033`）由加速度和角速度更新四元数；
  3. 默认由四元数计算欧拉角：`pit` 为绕 x 轴、`rol` 为绕 y 轴、`yaw` 为绕 z 轴的角度（rad）；
  4. 发布下表三个 Topic（发布时刻的时间戳）。
- 查询接口：`GetAccl()`、`GetGyro()`、`GetEuler()`、`GetQuaternion()`、`IsOnline()`、
  `GetTimestamp()`。当前实现中 `IsOnline()` 在收到第一帧后即为 `true`，离线判断只在接收
  回调里执行，因此不会因断线变回 `false`；`GetTimestamp()` 不被更新，始终为 0。

## Topic

Topic 名称固定，不可配置。

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `atomimu_eulr` | `AtomImuCan::Euler { pit, rol, yaw }` | 欧拉角，rad |
| `atomimu_absaccl` | `AtomImuCan::Vector3 { x, y, z }` | 去重力加速度，g |
| `atomimu_gyro` | `AtomImuCan::Vector3 { x, y, z }` | 角速度，rad/s |

## 依赖

无其他模块依赖，仅使用 LibXR。

## 构造接口

```cpp
AtomImuCan(LibXR::CAN& can_bus,
           const Param& param = {.can_id = 10});
```

依赖：

- `can_bus`：连接 AtomImu 的 `LibXR::CAN`。

配置（`Param` 字段）：

- `can_id`：AtomImu 的基础 CAN ID（标准帧），模块接收 `can_id` 到 `can_id + 4`，默认 10。

## 使用

```sh
xrobot module add QDU-Robomaster/AtomImuCan
xrobot setup
xrobot instance add QDU-Robomaster/AtomImuCan
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `can_bus` 填为 BSP 中用 `XR_REGISTER` 注册的 CAN 对象名：

```yaml
modules:
  - module: QDU-Robomaster/AtomImuCan
    id: atomimucan_0
    args:
      - can_bus: can1
      - param:
          can_id: '10'
```

BSP 侧：

```cpp
XR_REGISTER(can1, LibXR::CAN);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/AtomImuCan`
（在 BSP 中）打印 manifest 和当前的构造函数。

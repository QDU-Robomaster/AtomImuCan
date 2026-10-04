# AtomImuCan

AtomImu 的 CAN 接收模块：解码加速度与角速度帧，解算姿态并发布欧拉角、去重力加速度与角速度 / CAN receiver Module for the AtomImu that decodes acceleration and angular-velocity frames, estimates attitude, and publishes Euler angles, gravity-free acceleration and angular velocity

## 1. 模块作用 / Purpose

构造时，AtomImuCan 在 `can_bus` 上注册标准帧 ID 范围 `[can_id, can_id + 4]` 的接收回调，并创建线程 `AtomImuCan`（优先级 HIGH，栈深 1024）。接收回调按 `帧 ID - can_id` 的偏移区分帧类型：

- 偏移 0：加速度，三个 21 位字段，按 ±24 g 解码。
- 偏移 1：角速度，三个 21 位字段，按 ±2000 °/s 解码为 rad/s。
- 偏移 3（欧拉角）与偏移 4（四元数）：头文件宏 `USE_ATOMIMU_EULER` 与 `USE_ATOMIMU_QUATERNION` 为 1 时解码并使用；默认均为 0，欧拉角与四元数由本地解算。
- 其他偏移的帧被忽略。

线程每 2 ms 运行一轮：

1. 用上一轮的四元数估计重力方向，从加速度中减去，得到去重力加速度；
2. 默认以 Madgwick 风格的算法（`BETA_IMU = 0.033`）由加速度和角速度更新四元数；
3. 默认由四元数计算欧拉角，`pit` 为绕 x 轴、`rol` 为绕 y 轴、`yaw` 为绕 z 轴的角度，单位 rad；
4. 发布欧拉角、去重力加速度和角速度三个 Topic（见第 3 节）。

此外，`GetAccl()`、`GetGyro()`、`GetEuler()`、`GetQuaternion()` 返回最近一次的加速度、角速度、欧拉角和四元数。

Upon construction, AtomImuCan registers a receive callback on `can_bus` for the standard-frame ID range `[can_id, can_id + 4]` and creates the thread `AtomImuCan` (priority HIGH, stack depth 1024). The callback distinguishes the frame type by the offset `frame ID - can_id`:

- Offset 0: acceleration, three 21-bit fields decoded over ±24 g.
- Offset 1: angular velocity, three 21-bit fields decoded over ±2000 °/s into rad/s.
- Offset 3 (Euler angles) and offset 4 (quaternion): decoded and used when the header macros `USE_ATOMIMU_EULER` and `USE_ATOMIMU_QUATERNION` are 1; both default to 0, in which case the Euler angles and the quaternion are computed locally.
- Frames with other offsets are ignored.

The thread runs one iteration every 2 ms:

1. The gravity direction is estimated from the previous quaternion and subtracted from the acceleration, giving the gravity-free acceleration;
2. By default the quaternion is updated from the acceleration and angular velocity with a Madgwick-style algorithm (`BETA_IMU = 0.033`);
3. By default the Euler angles are computed from the quaternion: `pit` is the angle about the x axis, `rol` about the y axis and `yaw` about the z axis, in rad;
4. The Euler angles, the gravity-free acceleration and the angular velocity are published as three Topics (see section 3).

In addition, `GetAccl()`, `GetGyro()`, `GetEuler()` and `GetQuaternion()` return the latest acceleration, angular velocity, Euler angles and quaternion.

## 2. 构造接口 / Constructor

```cpp
AtomImuCan(LibXR::CAN& can_bus, const Param& param = {.can_id = 10});
```

依赖：

- `can_bus`：连接 AtomImu 的 `LibXR::CAN`。

配置参数（`Param`）：

- `can_id`：AtomImu 的基础 CAN ID（标准帧），模块接收 `can_id` 到 `can_id + 4` 的帧，默认 10。

Dependencies:

- `can_bus`: the `LibXR::CAN` connected to the AtomImu.

Configuration parameters (`Param`):

- `can_id`: base CAN ID (standard frame) of the AtomImu; the Module receives the frames from `can_id` to `can_id + 4`, default 10.

## 3. Topic

Topic 名称固定，模块发布以下三个 Topic。

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `atomimu_eulr` | `AtomImuCan::Euler { pit, rol, yaw }` | 欧拉角，单位 rad |
| `atomimu_absaccl` | `AtomImuCan::Vector3 { x, y, z }` | 去重力加速度，单位 g |
| `atomimu_gyro` | `AtomImuCan::Vector3 { x, y, z }` | 角速度，单位 rad/s |

The Topic names are fixed; the Module publishes the following three Topics.

| Topic | Type | Meaning |
| --- | --- | --- |
| `atomimu_eulr` | `AtomImuCan::Euler { pit, rol, yaw }` | Euler angles in rad |
| `atomimu_absaccl` | `AtomImuCan::Vector3 { x, y, z }` | Gravity-free acceleration in g |
| `atomimu_gyro` | `AtomImuCan::Vector3 { x, y, z }` | Angular velocity in rad/s |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/AtomImuCan` 写入的实例，`can_bus` 填为 CAN 对象的名称，该名称来自 BSP 的 `XR_REGISTER`（硬件注册）：

An instance written by `xrobot instance add QDU-Robomaster/AtomImuCan`, with `can_bus` set to the name of a CAN object, which comes from the BSP's `XR_REGISTER` (Registration):

```yaml
modules:
  - module: QDU-Robomaster/AtomImuCan
    id: atomimucan_0
    args:
      - can_bus: can1
      - param:
          can_id: 10
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：通过 CAN 总线发送加速度与角速度帧的 AtomImu，帧 ID 位于 `can_id` 到 `can_id + 4` 之间，由 BSP 注册为 `LibXR::CAN` 对象。

Dependencies: LibXR.

Hardware: an AtomImu sending acceleration and angular-velocity frames over CAN, with frame IDs from `can_id` to `can_id + 4`, registered by the BSP as a `LibXR::CAN` object.

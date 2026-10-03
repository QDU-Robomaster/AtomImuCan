#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: AtomImu 的 CAN 接收模块：解码加速度与角速度帧，解算姿态并发布欧拉角、去重力加速度与角速度 / CAN receiver Module for the AtomImu that decodes acceleration and angular-velocity frames, estimates attitude, and publishes Euler angles, gravity-free acceleration and angular velocity
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "thread.hpp"

#define ENCODER_21_MAX_INT ((1u << 21) - 1)
#define CAN_PACK_ID_ACCL 0
#define CAN_PACK_ID_GYRO 1
#define CAN_PACK_ID_EULR 3
#define CAN_PACK_ID_QUAT 4
#define CAN_PACK_ID_TIME 5
#define BETA_IMU (0.033f)

/* 1: 接收 CAN 四元数，0: 本地解算
 * 1: receive the quaternion over CAN, 0: compute locally */
#define USE_ATOMIMU_QUATERNION 0
/* 1: 接收 CAN 欧拉角，0: 本地解算
 * 1: receive the Euler angles over CAN, 0: compute locally */
#define USE_ATOMIMU_EULER 0

typedef union
{
  struct __attribute__((packed))
  {
    int32_t data1 : 21;
    int32_t data2 : 21;
    int32_t data3 : 21;
    int32_t res : 1;
  };
  struct __attribute__((packed))
  {
    uint32_t data1_unsigned : 21;
    uint32_t data2_unsigned : 21;
    uint32_t data3_unsigned : 21;
    uint32_t res_unsigned : 1;
  };
  uint8_t raw[8];
} CanData3;

typedef struct __attribute__((packed))
{
  union
  {
    int16_t data[4];
    uint16_t data_unsigned[4];
  };
} CanData4;

/**
 * @brief AtomImu 的 CAN 接收模块：解码加速度与角速度帧，解算姿态并发布 Topic。
 *        CAN receiver Module for the AtomImu: decodes acceleration and angular-velocity
 *        frames, estimates attitude, and publishes Topics.
 */
class AtomImuCan
{
 public:
  /**
   * @brief 模块配置参数。
   *        Module configuration parameters.
   */
  struct Param
  {
    uint16_t can_id;  ///< AtomImu 的基础 CAN ID（标准帧），接收 can_id 到 can_id + 4
                      ///< Base CAN ID (standard frame) of the AtomImu; frames from can_id
                      ///< to can_id + 4 are received
  };

  /**
   * @brief 三维向量。
   *        Three-dimensional vector.
   */
  struct Vector3
  {
    float x = 0.0f;  ///< x 分量 X component
    float y = 0.0f;  ///< y 分量 Y component
    float z = 0.0f;  ///< z 分量 Z component
  };

  /**
   * @brief 欧拉角 (rad)。
   *        Euler angles (rad).
   */
  struct Euler
  {
    float pit = 0.0f;  ///< 绕 x 轴的角度 Angle about the x axis
    float rol = 0.0f;  ///< 绕 y 轴的角度 Angle about the y axis
    float yaw = 0.0f;  ///< 绕 z 轴的角度 Angle about the z axis
  };

  /**
   * @brief 姿态四元数，q0 为标量部分。
   *        Attitude quaternion; q0 is the scalar part.
   */
  struct Quaternion
  {
    float q0 = -1.0f;  ///< 标量部分 Scalar part
    float q1 = 0.0f;   ///< x 向量部分 X vector part
    float q2 = 0.0f;   ///< y 向量部分 Y vector part
    float q3 = 0.0f;   ///< z 向量部分 Z vector part
  };

  /**
   * @brief 模块保存的最近一次测量与解算结果。
   *        Latest measurements and estimation results held by the Module.
   */
  struct Feedback
  {
    Vector3 accl;            ///< 加速度 (g) Acceleration (g)
    Vector3 accl_abs;        ///< 去重力加速度 (g) Gravity-free acceleration (g)
    Vector3 gyro;            ///< 角速度 (rad/s) Angular velocity (rad/s)
    Euler eulr;              ///< 欧拉角 (rad) Euler angles (rad)
    Quaternion quat;         ///< 姿态四元数 Attitude quaternion
    uint64_t timestamp = 0;  ///< 时间戳字段 Timestamp field
    bool online = false;     ///< 在线标志，接收到帧时置为 true
                             ///< Online flag, set to true when a frame is received
  };

  /**
   * @brief 构造 AtomImuCan，注册 CAN 接收回调并创建解算线程。
   *        Construct AtomImuCan, register the CAN receive callback and create the
   *        estimation thread.
   *
   * @param can_bus 连接 AtomImu 的 CAN 总线。
   *                CAN bus connected to the AtomImu.
   * @param param 配置参数，默认基础 CAN ID 为 10。
   *              Configuration parameters; the default base CAN ID is 10.
   */
  AtomImuCan(
      LibXR::CAN& can_bus,
      const Param& param = {.can_id = 10})
      : param_(param),
        feedback_{},
        atomimu_eulr_topic_(
            LibXR::Topic::CreateTopic<decltype(feedback_.eulr)>("atomimu_eulr")),
        atomimu_absaccl_topic_(
            LibXR::Topic::CreateTopic<decltype(feedback_.accl_abs)>("atomimu_absaccl")),
        atomimu_gyro_topic_(
            LibXR::Topic::CreateTopic<decltype(feedback_.gyro)>("atomimu_gyro")),
        can_(std::addressof(can_bus))
  {
    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, AtomImuCan* self, const LibXR::CAN::ClassicPack& pack)
        { RxCallback(in_isr, self, pack); }, this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, param_.can_id, param_.can_id + 4);

    thread_.Create(this, ThreadFunction, "AtomImuCan", 1024,
                   LibXR::Thread::Priority::HIGH);
  }

  /**
   * @brief 解算线程入口，每 2 ms 更新去重力加速度、四元数与欧拉角并发布 Topic。
   *        Estimation thread entry; every 2 ms it updates the gravity-free acceleration,
   *        quaternion and Euler angles and publishes the Topics.
   *
   * @param atomimu 模块实例。
   *                Module instance.
   */
  static void ThreadFunction(AtomImuCan* atomimu)
  {
    while (true)
    {
      auto last_time = LibXR::Timebase::GetMilliseconds();
      atomimu->CalcAbsAccl();

#if !USE_ATOMIMU_QUATERNION
      atomimu->CalQuat();
#endif

#if !USE_ATOMIMU_EULER
      atomimu->CalcEulr();
#endif

      atomimu->atomimu_eulr_topic_.Publish(atomimu->feedback_.eulr);
      atomimu->atomimu_absaccl_topic_.Publish(atomimu->feedback_.accl_abs);
      atomimu->atomimu_gyro_topic_.Publish(atomimu->feedback_.gyro);
      LibXR::Thread::SleepUntil(last_time, 2);
    }
  }

  /**
   * @brief 按 pack.id - can_id 的偏移解码一帧 CAN 数据。
   *        Decode one CAN frame by the offset pack.id - can_id.
   *
   * @param pack 接收到的 CAN 数据包。
   *             Received CAN packet.
   */
  void Decode(const LibXR::CAN::ClassicPack& pack)
  {
    uint32_t packet_type = pack.id - param_.can_id;
    switch (packet_type)
    {
      case CAN_PACK_ID_ACCL:
      {
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        feedback_.accl.x = DecodeFloat21(can_data->data1_unsigned, -24.0f, 24.0f);
        feedback_.accl.y = DecodeFloat21(can_data->data2_unsigned, -24.0f, 24.0f);
        feedback_.accl.z = DecodeFloat21(can_data->data3_unsigned, -24.0f, 24.0f);
        break;
      }

      case CAN_PACK_ID_GYRO:
      {
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        float min_gyro = -2000.0f * M_PI / 180.0f;
        float max_gyro = 2000.0f * M_PI / 180.0f;
        feedback_.gyro.x = DecodeFloat21(can_data->data1_unsigned, min_gyro, max_gyro);
        feedback_.gyro.y = DecodeFloat21(can_data->data2_unsigned, min_gyro, max_gyro);
        feedback_.gyro.z = DecodeFloat21(can_data->data3_unsigned, min_gyro, max_gyro);
        break;
      }

      case CAN_PACK_ID_EULR:
      {
#if USE_ATOMIMU_EULER
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        feedback_.eulr.pit = DecodeFloat21(can_data->data1_unsigned, -M_PI, M_PI);
        feedback_.eulr.rol = DecodeFloat21(can_data->data2_unsigned, -M_PI, M_PI);
        feedback_.eulr.yaw = DecodeFloat21(can_data->data3_unsigned, -M_PI, M_PI);
#endif
        break;
      }

      case CAN_PACK_ID_QUAT:
      {
#if USE_ATOMIMU_QUATERNION
        const CanData4* can_data = reinterpret_cast<const CanData4*>(pack.data);
        feedback_.quat.q0 = DecodeInt16Normalized(can_data->data[0]);
        feedback_.quat.q1 = DecodeInt16Normalized(can_data->data[1]);
        feedback_.quat.q2 = DecodeInt16Normalized(can_data->data[2]);
        feedback_.quat.q3 = DecodeInt16Normalized(can_data->data[3]);
        quat_ = feedback_.quat;
#endif
        break;
      }
      default:
        break;
    }
  }

  /**
   * @brief 由当前四元数估计重力方向，从加速度中减去并写入 accl_abs。
   *        Estimate the gravity direction from the current quaternion, subtract it from
   *        the acceleration and store the result in accl_abs.
   */
  void CalcAbsAccl()
  {
    float gravity_b[3];

    gravity_b[0] = 2.0f * ((quat_.q1 * quat_.q3 - quat_.q0 * quat_.q2) * 1.0f);

    gravity_b[1] = 2.0f * ((quat_.q2 * quat_.q3 + quat_.q0 * quat_.q1) * 1.0f);

    gravity_b[2] = 2.0f * ((0.5f - quat_.q1 * quat_.q1 - quat_.q2 * quat_.q2) * 1.0f);

    feedback_.accl_abs.x = feedback_.accl.x - gravity_b[0];
    feedback_.accl_abs.y = feedback_.accl.y - gravity_b[1];
    feedback_.accl_abs.z = feedback_.accl.z - gravity_b[2];
  }

  /**
   * @brief 用加速度和角速度以 Madgwick 风格的算法更新四元数；时间步长取自两次调用的间隔。
   *        Update the quaternion from the acceleration and angular velocity with a
   *        Madgwick-style algorithm; the time step is the interval between two calls.
   */
  void CalQuat()
  {
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float q_2q0, q_2q1, q_2q2, q_2q3, q_4q0, q_4q1, q_4q2, q_8q1, q_8q2, q0q0, q1q1, q2q2,
        q3q3;

    now_ = LibXR::Timebase::GetMicroseconds();
    dt_ = (now_ - last_wakeup_) / 1000000.0f;
    last_wakeup_ = now_;

    float ax = feedback_.accl.x;
    float ay = feedback_.accl.y;
    float az = feedback_.accl.z;

    float gx = feedback_.gyro.x;
    float gy = feedback_.gyro.y;
    float gz = feedback_.gyro.z;

    q_dot1 = 0.5f * (-this->quat_.q1 * gx - this->quat_.q2 * gy - this->quat_.q3 * gz);
    q_dot2 = 0.5f * (this->quat_.q0 * gx + this->quat_.q2 * gz - this->quat_.q3 * gy);
    q_dot3 = 0.5f * (this->quat_.q0 * gy - this->quat_.q1 * gz + this->quat_.q3 * gx);
    q_dot4 = 0.5f * (this->quat_.q0 * gz + this->quat_.q1 * gy - this->quat_.q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
      recip_norm = InvSqrtf(ax * ax + ay * ay + az * az);
      ax *= recip_norm;
      ay *= recip_norm;
      az *= recip_norm;

      q_2q0 = 2.0f * this->quat_.q0;
      q_2q1 = 2.0f * this->quat_.q1;
      q_2q2 = 2.0f * this->quat_.q2;
      q_2q3 = 2.0f * this->quat_.q3;
      q_4q0 = 4.0f * this->quat_.q0;
      q_4q1 = 4.0f * this->quat_.q1;
      q_4q2 = 4.0f * this->quat_.q2;
      q_8q1 = 8.0f * this->quat_.q1;
      q_8q2 = 8.0f * this->quat_.q2;
      q0q0 = this->quat_.q0 * this->quat_.q0;
      q1q1 = this->quat_.q1 * this->quat_.q1;
      q2q2 = this->quat_.q2 * this->quat_.q2;
      q3q3 = this->quat_.q3 * this->quat_.q3;

      s0 = q_4q0 * q2q2 + q_2q2 * ax + q_4q0 * q1q1 - q_2q1 * ay;
      s1 = q_4q1 * q3q3 - q_2q3 * ax + 4.0f * q0q0 * this->quat_.q1 - q_2q0 * ay - q_4q1 +
           q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
      s2 = 4.0f * q0q0 * this->quat_.q2 + q_2q0 * ax + q_4q2 * q3q3 - q_2q3 * ay - q_4q2 +
           q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
      s3 = 4.0f * q1q1 * this->quat_.q3 - q_2q1 * ax + 4.0f * q2q2 * this->quat_.q3 -
           q_2q2 * ay;

      recip_norm = InvSqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

      s0 *= recip_norm;
      s1 *= recip_norm;
      s2 *= recip_norm;
      s3 *= recip_norm;

      q_dot1 -= BETA_IMU * s0;
      q_dot2 -= BETA_IMU * s1;
      q_dot3 -= BETA_IMU * s2;
      q_dot4 -= BETA_IMU * s3;
    }

    this->quat_.q0 += q_dot1 * this->dt_;
    this->quat_.q1 += q_dot2 * this->dt_;
    this->quat_.q2 += q_dot3 * this->dt_;
    this->quat_.q3 += q_dot4 * this->dt_;

    recip_norm =
        InvSqrtf(this->quat_.q0 * this->quat_.q0 + this->quat_.q1 * this->quat_.q1 +
                 this->quat_.q2 * this->quat_.q2 + this->quat_.q3 * this->quat_.q3);
    this->quat_.q0 *= recip_norm;
    this->quat_.q1 *= recip_norm;
    this->quat_.q2 *= recip_norm;
    this->quat_.q3 *= recip_norm;
    feedback_.quat = quat_;
  }

  /**
   * @brief 由当前四元数计算欧拉角。
   *        Compute the Euler angles from the current quaternion.
   */
  void CalcEulr()
  {
    const float SINR_COSP = 2.0f * (quat_.q0 * quat_.q1 + quat_.q2 * quat_.q3);
    const float COSR_COSP = 1.0f - 2.0f * (quat_.q1 * quat_.q1 + quat_.q2 * quat_.q2);
    feedback_.eulr.pit = atan2f(SINR_COSP, COSR_COSP);

    const float SINP = 2.0f * (quat_.q0 * quat_.q2 - quat_.q3 * quat_.q1);

    if (fabsf(SINP) >= 1.0f)
    {
      feedback_.eulr.rol = copysignf(M_PI / 2.0f, SINP);
    }
    else
    {
      feedback_.eulr.rol = asinf(SINP);
    }

    const float SINY_COSP = 2.0f * (quat_.q0 * quat_.q3 + quat_.q1 * quat_.q2);
    const float COSY_COSP = 1.0f - 2.0f * (quat_.q2 * quat_.q2 + quat_.q3 * quat_.q3);
    feedback_.eulr.yaw = atan2f(SINY_COSP, COSY_COSP);
  }

  /**
   * @brief 获取最近一次的加速度。
   *        Get the latest acceleration.
   * @return 加速度 (g)。
   *         Acceleration (g).
   */
  Vector3 GetAccl() const { return feedback_.accl; }

  /**
   * @brief 获取最近一次的角速度。
   *        Get the latest angular velocity.
   * @return 角速度 (rad/s)。
   *         Angular velocity (rad/s).
   */
  Vector3 GetGyro() const { return feedback_.gyro; }

  /**
   * @brief 获取最近一次的欧拉角。
   *        Get the latest Euler angles.
   * @return 欧拉角 (rad)。
   *         Euler angles (rad).
   */
  Euler GetEuler() const { return feedback_.eulr; }

  /**
   * @brief 获取最近一次的姿态四元数。
   *        Get the latest attitude quaternion.
   * @return 姿态四元数。
   *         Attitude quaternion.
   */
  Quaternion GetQuaternion() const { return feedback_.quat; }

  /**
   * @brief 获取反馈中的时间戳字段。
   *        Get the timestamp field of the feedback.
   * @return 时间戳字段的值。
   *         Value of the timestamp field.
   */
  uint64_t GetTimestamp() const { return feedback_.timestamp; }

  /**
   * @brief 获取反馈中的在线标志。
   *        Get the online flag of the feedback.
   * @return 在线标志，接收到帧后为 true。
   *         Online flag, true after a frame has been received.
   */
  bool IsOnline() const { return feedback_.online; }

 private:
  static float DecodeFloat21(uint32_t encoded, float min, float max)
  {
    float norm = static_cast<float>(encoded & ENCODER_21_MAX_INT) /
                 static_cast<float>(ENCODER_21_MAX_INT);
    return min + norm * (max - min);
  }

  static float DecodeInt16Normalized(int16_t value)
  {
    return static_cast<float>(value) / static_cast<float>(INT16_MAX);
  }

  float InvSqrtf(float x) { return 1.0f / sqrtf(x); }

  void CheckOffline()
  {
    uint64_t current_time = LibXR::Timebase::GetMicroseconds();
    if (current_time - last_online_time_ > 100000)
    { /* 100ms超时 */
      feedback_.online = false;
    }
  }

  /**
   * @brief CAN 接收回调的静态包装函数。
   *        Static wrapper of the CAN receive callback.
   *
   * @param in_isr 是否在中断服务程序中调用。
   *               Whether called from an interrupt service routine.
   * @param self 模块实例。
   *             Module instance.
   * @param pack 接收到的 CAN 数据包。
   *             Received CAN packet.
   */
  static void RxCallback(bool in_isr, AtomImuCan* self,
                         const LibXR::CAN::ClassicPack& pack)
  {
    UNUSED(in_isr);
    self->Decode(pack);
    self->feedback_.online = true;
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    self->CheckOffline();
  }

  uint64_t last_online_time_ = 0;

  Param param_;
  Feedback feedback_;
  Quaternion quat_;

  float dt_ = 0;
  uint64_t now_ = 0;
  uint64_t last_wakeup_ = 0;

  LibXR::Topic atomimu_eulr_topic_;
  LibXR::Topic atomimu_absaccl_topic_;
  LibXR::Topic atomimu_gyro_topic_;
  LibXR::CAN* can_;
  LibXR::Thread thread_;
};

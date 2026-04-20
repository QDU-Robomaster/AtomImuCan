#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: AtomImu Communication(can)
constructor_args:
 - param:
     can_id: 10
     can_bus_name: can2
template_args: []
required_hardware:
  -can
depends: []
=== END MANIFEST === */
// clang-format on

#include <utility>

#include "message.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_type.hpp"
#include "thread.hpp"

#define ENCODER_21_MAX_INT ((1u << 21) - 1)
#define CAN_PACK_ID_ACCL 0
#define CAN_PACK_ID_GYRO 1
#define CAN_PACK_ID_EULR 3
#define CAN_PACK_ID_QUAT 4
#define CAN_PACK_ID_TIME 5
#define BETA_IMU (0.033f)

/* 四元数和欧拉角接收控制 */
#define USE_ATOMIMU_QUATERNION 0  /* 1: 接收CAN四元数, 0: 使用自己计算 */
#define USE_ATOMIMU_EULER 0       /* 1: 接收CAN欧拉角, 0: 使用自己计算 */

typedef union {
  struct __attribute__((packed)) {
    int32_t data1 : 21;
    int32_t data2 : 21;
    int32_t data3 : 21;
    int32_t res : 1;
  };
  struct __attribute__((packed)) {
    uint32_t data1_unsigned : 21;
    uint32_t data2_unsigned : 21;
    uint32_t data3_unsigned : 21;
    uint32_t res_unsigned : 1;
  };
  uint8_t raw[8];
} CanData3;

typedef struct __attribute__((packed)) {
  union {
    int16_t data[4];
    uint16_t data_unsigned[4];
  };
} CanData4;

class AtomImuCan : public LibXR::Application {
 public:
  /*陀螺仪参数*/
  struct Param {
    uint16_t can_id;
    const char* can_bus_name;
  };

  struct Vector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
  };

  struct Euler {
    float pit = 0.0f;
    float rol = 0.0f;
    float yaw = 0.0f;
  };

  struct Quaternion {
    float q0 = -1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
  };

  struct Feedback {
    Vector3 accl;
    Vector3 accl_abs;
    Vector3 gyro;
    Euler eulr;
    Quaternion quat;
    uint64_t timestamp = 0;
    bool online = false;
  };

  /**
   * @brief IMU 的构造函数
   * @param hw
   * @param app
   * @param param 陀螺仪参数 (CANID CanBusName 名称前缀)
   */
  AtomImuCan(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             Param&& param)
      : param_(std::forward<Param>(param)),
        feedback_{},
        atomimu_eulr_topic_("atomimu_eulr", sizeof(feedback_.eulr)),
        atomimu_absaccl_topic_("atomimu_absaccl", sizeof(feedback_.accl_abs)),
        atomimu_gyro_topic_("atomimu_gyro", sizeof(feedback_.gyro)),
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, AtomImuCan* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE,
                   param_.can_id, param_.can_id + 4);

    thread_.Create(this,ThreadFunction,"AtomImuCan",
    1024,LibXR::Thread::Priority::HIGH);
  }

static void ThreadFunction(AtomImuCan *atomimu){
    while(true){
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
      LibXR::Thread::SleepUntil(last_time,2);
    }
  }


  void Decode(const LibXR::CAN::ClassicPack& pack) {
    uint32_t packet_type = pack.id - param_.can_id;
    switch (packet_type) {
      case CAN_PACK_ID_ACCL: {
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        feedback_.accl.x =
            DecodeFloat21(can_data->data1_unsigned, -24.0f, 24.0f);
        feedback_.accl.y =
            DecodeFloat21(can_data->data2_unsigned, -24.0f, 24.0f);
        feedback_.accl.z =
            DecodeFloat21(can_data->data3_unsigned, -24.0f, 24.0f);
        break;
      }

      case CAN_PACK_ID_GYRO: {
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        float min_gyro = -2000.0f * M_PI / 180.0f;
        float max_gyro = 2000.0f * M_PI / 180.0f;
        feedback_.gyro.x =
            DecodeFloat21(can_data->data1_unsigned, min_gyro, max_gyro);
        feedback_.gyro.y =
            DecodeFloat21(can_data->data2_unsigned, min_gyro, max_gyro);
        feedback_.gyro.z =
            DecodeFloat21(can_data->data3_unsigned, min_gyro, max_gyro);
        break;
      }

      case CAN_PACK_ID_EULR: {
#if USE_ATOMIMU_EULER
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        feedback_.eulr.pit = DecodeFloat21(can_data->data1_unsigned, -M_PI, M_PI);
        feedback_.eulr.rol = DecodeFloat21(can_data->data2_unsigned, -M_PI, M_PI);
        feedback_.eulr.yaw = DecodeFloat21(can_data->data3_unsigned, -M_PI, M_PI);
#endif
        break;
      }

      case CAN_PACK_ID_QUAT: {
#if USE_ATOMIMU_QUATERNION
        const CanData4* can_data = reinterpret_cast<const CanData4*>(pack.data);
        feedback_.quat.q0 = DecodeInt16Normalized(can_data->data[0]);
        feedback_.quat.q1 = DecodeInt16Normalized(can_data->data[1]);
        feedback_.quat.q2 = DecodeInt16Normalized(can_data->data[2]);
        feedback_.quat.q3 = DecodeInt16Normalized(can_data->data[3]);
#endif
        break;
      }
      default:
        break;
    }
  }

  /*去除重力加速度*/
  void CalcAbsAccl() {
    float gravity_b[3];

/*去除重力加速度*/
void CalcAbsAccl()
{
  float gravity_b[3];

  gravity_b[0] = 2.0f * ((quat_.q1 * quat_.q3 - quat_.q0 * quat_.q2) * 1.0f);

  gravity_b[1] = 2.0f * ((quat_.q2 * quat_.q3 + quat_.q0 * quat_.q1) * 1.0f);

  gravity_b[2] =
      2.0f * ((0.5f - quat_.q1 * quat_.q1 - quat_.q2 * quat_.q2) * 1.0f);

  feedback_.accl_abs.x = feedback_.accl.x - gravity_b[0];
  feedback_.accl_abs.y = feedback_.accl.y - gravity_b[1];
  feedback_.accl_abs.z = feedback_.accl.z - gravity_b[2];
}

/* 计算四元数 在不接收四元数的时候使用 */
void CalQuat() {

    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float q_2q0, q_2q1, q_2q2, q_2q3, q_4q0, q_4q1, q_4q2, q_8q1, q_8q2, q0q0,
        q1q1, q2q2, q3q3;

    now_ = LibXR::Timebase::GetMicroseconds();
    dt_ = (now_ - last_wakeup_) / 1000000.0f;
    last_wakeup_ = now_;

  float ax = feedback_.accl.x;
  float ay = feedback_.accl.y;
  float az = feedback_.accl.z;

  float gx = feedback_.gyro.x;
  float gy = feedback_.gyro.y;
  float gz = feedback_.gyro.z;

  q_dot1 =
      0.5f * (-this->quat_.q1 * gx - this->quat_.q2 * gy - this->quat_.q3 * gz);
  q_dot2 =
      0.5f * (this->quat_.q0 * gx + this->quat_.q2 * gz - this->quat_.q3 * gy);
  q_dot3 =
      0.5f * (this->quat_.q0 * gy - this->quat_.q1 * gz + this->quat_.q3 * gx);
  q_dot4 =
      0.5f * (this->quat_.q0 * gz + this->quat_.q1 * gy - this->quat_.q2 * gx);


  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

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
    s1 = q_4q1 * q3q3 - q_2q3 * ax + 4.0f * q0q0 * this->quat_.q1 - q_2q0 * ay -
         q_4q1 + q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
    s2 = 4.0f * q0q0 * this->quat_.q2 + q_2q0 * ax + q_4q2 * q3q3 - q_2q3 * ay -
         q_4q2 + q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
    s3 = 4.0f * q1q1 * this->quat_.q3 - q_2q1 * ax +
         4.0f * q2q2 * this->quat_.q3 - q_2q2 * ay;


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

  recip_norm = InvSqrtf(
      this->quat_.q0 * this->quat_.q0 + this->quat_.q1 * this->quat_.q1 +
      this->quat_.q2 * this->quat_.q2 + this->quat_.q3 * this->quat_.q3);
  this->quat_.q0 *= recip_norm;
  this->quat_.q1 *= recip_norm;
  this->quat_.q2 *= recip_norm;
  this->quat_.q3 *= recip_norm;
}

/*计算欧拉角 在不接收欧拉角的时候使用*/
void CalcEulr() {

  const float SINR_COSP = 2.0f * (quat_.q0 * quat_.q1 +
                                  quat_.q2 * quat_.q3);
  const float COSR_COSP = 1.0f - 2.0f * (quat_.q1 * quat_.q1 +
                                         quat_.q2 * quat_.q2);
  feedback_.eulr.pit = atan2f(SINR_COSP, COSR_COSP);

  const float SINP = 2.0f * (quat_.q0 * quat_.q2 -
                             quat_.q3 * quat_.q1);

  if (fabsf(SINP) >= 1.0f) {
    feedback_.eulr.rol = copysignf(M_PI / 2.0f, SINP);
  } else {
    feedback_.eulr.rol = asinf(SINP);
  }

  const float SINY_COSP = 2.0f * (quat_.q0 * quat_.q3 +
                                  quat_.q1 * quat_.q2);
  const float COSY_COSP = 1.0f - 2.0f * (quat_.q2 * quat_.q2 +
                                         quat_.q3 * quat_.q3);
  feedback_.eulr.yaw = atan2f(SINY_COSP, COSY_COSP);

}

  Vector3 GetAccl() const { return feedback_.accl; }
  Vector3 GetGyro() const { return feedback_.gyro; }
  Euler GetEuler() const { return feedback_.eulr; }
  Quaternion GetQuaternion() const { return feedback_.quat; }
  uint64_t GetTimestamp() const { return feedback_.timestamp; }
  bool IsOnline() const { return feedback_.online; }

  void OnMonitor() override {}

 private:
  static float DecodeFloat21(uint32_t encoded, float min, float max) {
    float norm = static_cast<float>(encoded & ENCODER_21_MAX_INT) /
                 static_cast<float>(ENCODER_21_MAX_INT);
    return min + norm * (max - min);
  }

  static float DecodeInt16Normalized(int16_t value) {
    return static_cast<float>(value) / static_cast<float>(INT16_MAX);
  }

float InvSqrtf(float x) {
  return 1.0f / sqrtf(x);
}

  void CheckOffline() {
    uint64_t current_time = LibXR::Timebase::GetMicroseconds();
    if (current_time - last_online_time_ > 100000) { /* 100ms超时 */
      feedback_.online = false;
    }
  }

  /**
   * @brief CAN 接收回调的静态包装函数
   * @param in_isr 指示是否在中断服务程序中调用
   * @param self
   * @param pack 接收到的 CAN 数据包
   */
  static void RxCallback(bool in_isr, AtomImuCan* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
      self->Decode(pack);
      self->feedback_.online = true;
      self->last_online_time_ = LibXR::Timebase::GetMicroseconds();
      self->CheckOffline();
    }

  uint64_t last_online_time_ = 0; /* 方便查看陀螺仪是否在线 */

  Param param_;
  Feedback feedback_;
  Quaternion quat_;

  float dt_ = 0;
  uint64_t now_=0;
  uint64_t last_wakeup_ = 0;

  LibXR::Topic atomimu_eulr_topic_;
  LibXR::Topic atomimu_absaccl_topic_;
  LibXR::Topic atomimu_gyro_topic_;
  LibXR::CAN* can_;
  LibXR::Thread thread_;
};

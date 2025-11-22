#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: AtomImu Communication(can)
constructor_args:
 - param:
     can_id: 12
     can_bus_name: can1
template_args: []
required_hardware:
  -can
depends: []
=== END MANIFEST === */
// clang-format on


#include <utility>
#include "message.hpp"
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "thread.hpp"

#define ENCODER_21_MAX_INT ((1u << 21) - 1)
#define CAN_PACK_ID_ACCL 0
#define CAN_PACK_ID_GYRO 1
#define CAN_PACK_ID_EULR 3
#define CAN_PACK_ID_QUAT 4
#define CAN_PACK_ID_TIME 5


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

  /*传感器数据结构*/
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
    float q0 = 0.0f;
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
    : param_(std::forward<Param>(param)), feedback_{},
      atomimu_eulr_topic_("atomimu_eulr",sizeof(feedback_.eulr)),
      atomimu_absaccl_topic_("atomimu_absaccl",sizeof(feedback_.accl_abs)),
      atomimu_gyro_topic_("atomimu_gyro",sizeof(feedback_.gyro)),
      can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, AtomImuCan* self, const LibXR::CAN::ClassicPack& pack) {
            RxCallback(in_isr, self, pack);}, this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE,
                   param_.can_id, param_.can_id + 5);

    thread_.Create(this,ThreadFunction,"AtomImuCan",
    2048,LibXR::Thread::Priority::MEDIUM);
  }

static void ThreadFunction(AtomImuCan *atomimu){
    while(true){
    atomimu->CalcAbsAccl();
    atomimu->CalcEulr();
    atomimu->atomimu_eulr_topic_.Publish(atomimu->feedback_.eulr);
    atomimu->atomimu_absaccl_topic_.Publish(atomimu->feedback_.accl_abs);
    atomimu->atomimu_gyro_topic_.Publish(atomimu->feedback_.gyro);
    LibXR::Thread::Sleep(2);
    }
}

  void Decode(const LibXR::CAN::ClassicPack& pack) {
    uint32_t packet_type = pack.id - param_.can_id;
    switch (packet_type) {
      case CAN_PACK_ID_ACCL: {
        // 加速度计数据: ±24g范围
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        feedback_.accl.x = DecodeFloat21(can_data->data1_unsigned, -24.0f, 24.0f);
        feedback_.accl.y = DecodeFloat21(can_data->data2_unsigned, -24.0f, 24.0f);
        feedback_.accl.z = DecodeFloat21(can_data->data3_unsigned, -24.0f, 24.0f);
        break;
      }

      case CAN_PACK_ID_GYRO: {
        // 陀螺仪数据: ±2000 deg/s转换为rad/s
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        float min_gyro = -2000.0f * M_PI / 180.0f;
        float max_gyro = 2000.0f * M_PI / 180.0f;
        feedback_.gyro.x = DecodeFloat21(can_data->data1_unsigned, min_gyro, max_gyro);
        feedback_.gyro.y = DecodeFloat21(can_data->data2_unsigned, min_gyro, max_gyro);
        feedback_.gyro.z = DecodeFloat21(can_data->data3_unsigned, min_gyro, max_gyro);
        break;
      }

      case CAN_PACK_ID_EULR: {
        // 欧拉角: ±π rad
        eulr_receive_flag_=1;
        const CanData3* can_data = reinterpret_cast<const CanData3*>(pack.data);
        feedback_.eulr.pit = DecodeFloat21(can_data->data1_unsigned, -M_PI, M_PI);
        feedback_.eulr.rol = DecodeFloat21(can_data->data2_unsigned, -M_PI, M_PI);
        feedback_.eulr.yaw = DecodeFloat21(can_data->data3_unsigned, -M_PI, M_PI);
        break;
      }

      case CAN_PACK_ID_QUAT: {
        // 四元数数据: 归一化int16
        const CanData4* can_data = reinterpret_cast<const CanData4*>(pack.data);
        feedback_.quat.q0 = DecodeInt16Normalized(can_data->data[0]);
        feedback_.quat.q1 = DecodeInt16Normalized(can_data->data[1]);
        feedback_.quat.q2 = DecodeInt16Normalized(can_data->data[2]);
        feedback_.quat.q3 = DecodeInt16Normalized(can_data->data[3]);
        break;
      }
      default:
       break;
    }
  }


/*去除重力加速度*/
void CalcAbsAccl()
{
  float gravity_b[3];

  gravity_b[0] = 2.0f * ((feedback_.quat.q1 * feedback_.quat.q3 - feedback_.quat.q0 * feedback_.quat.q2) * 1.0f);

  gravity_b[1] = 2.0f * ((feedback_.quat.q2 * feedback_.quat.q3 + feedback_.quat.q0 * feedback_.quat.q1) * 1.0f);

  gravity_b[2] =
      2.0f * ((0.5f - feedback_.quat.q1 * feedback_.quat.q1 - feedback_.quat.q2 * feedback_.quat.q2) * 1.0f);

  feedback_.accl_abs.x = feedback_.accl.x - gravity_b[0];
  feedback_.accl_abs.y = feedback_.accl.y - gravity_b[1];
  feedback_.accl_abs.z = feedback_.accl.z - gravity_b[2];
}


/*计算欧拉角 在不接收欧拉角的时候使用*/
void CalcEulr() {
  if (eulr_receive_flag_ != 1){
  const float SINR_COSP = 2.0f * (feedback_.quat.q0 * feedback_.quat.q1 +
                                  feedback_.quat.q2 * feedback_.quat.q3);
  const float COSR_COSP = 1.0f - 2.0f * (feedback_.quat.q1 * feedback_.quat.q1 +
                                         feedback_.quat.q2 * feedback_.quat.q2);
  feedback_.eulr.pit = atan2f(SINR_COSP, COSR_COSP);

  const float SINP = 2.0f * (feedback_.quat.q0 * feedback_.quat.q2 -
                             feedback_.quat.q3 * feedback_.quat.q1);

  if (fabsf(SINP) >= 1.0f) {
    feedback_.eulr.rol = copysignf(M_PI / 2.0f, SINP);
  } else {
    feedback_.eulr.rol = asinf(SINP);
  }

  const float SINY_COSP = 2.0f * (feedback_.quat.q0 * feedback_.quat.q3 +
                                  feedback_.quat.q1 * feedback_.quat.q2);
  const float COSY_COSP = 1.0f - 2.0f * (feedback_.quat.q2 * feedback_.quat.q2 +
                                         feedback_.quat.q3 * feedback_.quat.q3);
  feedback_.eulr.yaw = atan2f(SINY_COSP, COSY_COSP);
  }
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


  void CheckOffline() {
    uint64_t current_time = LibXR::Timebase::GetMicroseconds();
    if (current_time - last_online_time_ > 100000) { /* 100ms超时 */
      feedback_.online = false;
    }
  }

  /**
   * @brief CAN 接收回调的静态包装函数
   * @param in_isr 指示是否在中断服务程序中调用
   * @param self 用户提供的参数，这里是 IMU 实例的指针
   * @param pack 接收到的 CAN 数据包
   */
  static void RxCallback(bool in_isr, AtomImuCan* self, const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
      self->Decode(pack);
      self->feedback_.online = true;
      self->last_online_time_ = LibXR::Timebase::GetMicroseconds();
      self->CheckOffline();
    }


  uint64_t last_online_time_ = 0; /* 方便查看陀螺仪是否在线 */
  bool eulr_receive_flag_ = 0;
  Param param_;
  Feedback feedback_;
  LibXR::Topic atomimu_eulr_topic_;
  LibXR::Topic atomimu_absaccl_topic_;
  LibXR::Topic atomimu_gyro_topic_;
  LibXR::CAN* can_;
  LibXR::Thread thread_;

};

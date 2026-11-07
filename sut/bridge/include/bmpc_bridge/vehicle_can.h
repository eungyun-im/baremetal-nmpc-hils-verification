// 안전 제어기 통합 (bare_metal_mpc/integ/bmpc_safety) 용 CAN 프레임.
//   PC -> MCU  차량 CAN (팀 capstone_hils dbc/vehicle.dbc): MORAI 차량 상태를 센서 ECU 가 낸 것처럼 만든다
//   MCU -> PC  SAFE_OUT 0x200, SAFE_STAT 0x201 (dbc/hils.dbc): 안전 제어기가 검사한 최종 명령과 상태
// 모든 프레임은 Alive 4 비트 + E2E CRC (CRC8 SAE-J1850, Data ID 하위·상위 바이트 다음 data[0..6]) 를 쓴다.
#ifndef BMPC_BRIDGE_VEHICLE_CAN_H
#define BMPC_BRIDGE_VEHICLE_CAN_H

#include <cmath>
#include <cstdint>
#include <cstring>

#include "bmpc/bmpc_protocol.h"

namespace bmpc_bridge {
namespace vcan {

constexpr uint32_t ID_ESC_SPEED = 0x1A0, ID_SAS_STEER = 0x1B0, ID_VCU_PEDAL = 0x1C0, ID_ACU_IMU = 0x1D0;
constexpr uint32_t ID_SAFE_OUT = 0x200, ID_SAFE_STAT = 0x201, ID_BENCH_RESET = 0x7F0;
constexpr uint16_t DID_ESC = 0x0C01, DID_SAS = 0x0C02, DID_PEDAL = 0x0C03, DID_IMU = 0x0C05;
constexpr uint16_t DID_SAFE_OUT = 0x0A03, DID_SAFE_STAT = 0x0A04;

inline uint8_t crc8_e2e(uint16_t data_id, const uint8_t *d)
{
  uint8_t c = 0xFF;
  auto step = [&c](uint8_t b) {
    c ^= b;
    for (int k = 0; k < 8; ++k) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x1D) : (uint8_t)(c << 1);
  };
  step((uint8_t)(data_id & 0xFF));
  step((uint8_t)(data_id >> 8));
  for (int i = 0; i < 7; ++i) step(d[i]);
  return (uint8_t)(c ^ 0xFF);
}

// 물리값 / scale 을 반올림해 범위로 포화 (팀 vehicle_can.py 의 floor(x/scale+0.5) 와 같다)
inline int32_t quant(double v, double scale, int32_t lo, int32_t hi)
{
  if (!std::isfinite(v)) v = 0.0;
  const double r = std::floor(v / scale + 0.5);
  return r < lo ? lo : (r > hi ? hi : (int32_t)r);
}
inline void put16(uint8_t *p, int32_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}
inline int16_t get_i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

inline void seal(bmpc_can_frame_t &f, uint32_t id, uint16_t did, uint8_t &alive)
{
  f.id = id;
  f.dlc = 8;
  f.data[6] = (uint8_t)(alive & 0x0F);
  alive = (uint8_t)((alive + 1) & 0x0F);
  f.data[7] = crc8_e2e(did, f.data);
}

struct Encoder {
  uint8_t alive_esc = 0, alive_sas = 0, alive_pedal = 0, alive_imu = 0;
  // ESC_SPEED: VehSpeed 0.01 km/h u16, LongAccel / LatAccel 0.001 m/s2 i16
  bmpc_can_frame_t esc(double speed_mps, double long_acc, double lat_acc)
  {
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    put16(&f.data[0], quant(speed_mps * 3.6, 0.01, 0, 65535));
    put16(&f.data[2], quant(long_acc, 0.001, -32768, 32767));
    put16(&f.data[4], quant(lat_acc, 0.001, -32768, 32767));
    seal(f, ID_ESC_SPEED, DID_ESC, alive_esc);
    return f;
  }
  // SAS_STEER: WheelAngle / FrontSteer / RearSteer 0.01 deg i16
  bmpc_can_frame_t sas(double wheel_deg)
  {
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    put16(&f.data[0], quant(wheel_deg, 0.01, -32768, 32767));
    put16(&f.data[2], quant(wheel_deg, 0.01, -32768, 32767));
    seal(f, ID_SAS_STEER, DID_SAS, alive_sas);
    return f;
  }
  // VCU_PEDAL: AccelPedal / BrakePedal 0.4 % u8
  bmpc_can_frame_t pedal(double accel01, double brake01)
  {
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    f.data[0] = (uint8_t)quant(accel01 * 100.0, 0.4, 0, 250);
    f.data[1] = (uint8_t)quant(brake01 * 100.0, 0.4, 0, 250);
    seal(f, ID_VCU_PEDAL, DID_PEDAL, alive_pedal);
    return f;
  }
  // ACU_IMU: YawRate 0.01 deg/s i16, ImuLongAcc / ImuLatAcc 0.001 m/s2 i16
  bmpc_can_frame_t imu(double yaw_rate_dps, double long_acc, double lat_acc)
  {
    bmpc_can_frame_t f;
    std::memset(&f, 0, sizeof(f));
    put16(&f.data[0], quant(yaw_rate_dps, 0.01, -32768, 32767));
    put16(&f.data[2], quant(long_acc, 0.001, -32768, 32767));
    put16(&f.data[4], quant(lat_acc, 0.001, -32768, 32767));
    seal(f, ID_ACU_IMU, DID_IMU, alive_imu);
    return f;
  }
};

struct SafeOut {
  double steer_deg = 0, accel_mps2 = 0;
  int state = 0, rx_age_10ms = 0, flags = 0, alive = 0, bus_off = 0;
};
struct SafeStat {
  int flags2 = 0, ctrl_alive_echo = -1, inputs_fresh = 0, alive = 0;
  double steer_track_err_deg = 0, lat_acc_meas = 0;
};
inline bool decode_out(const bmpc_can_frame_t &f, SafeOut *o)
{
  if (f.id != ID_SAFE_OUT || f.dlc != 8 || crc8_e2e(DID_SAFE_OUT, f.data) != f.data[7]) return false;
  o->steer_deg = get_i16(&f.data[0]) * 0.01;
  o->accel_mps2 = get_i16(&f.data[2]) * 0.01;
  o->state = f.data[4] & 0x0F;
  o->rx_age_10ms = f.data[4] >> 4;
  o->flags = f.data[5];
  o->alive = f.data[6] & 0x0F;
  o->bus_off = (f.data[6] >> 4) & 1;
  return true;
}
inline bool decode_stat(const bmpc_can_frame_t &f, SafeStat *s)
{
  if (f.id != ID_SAFE_STAT || f.dlc != 8 || crc8_e2e(DID_SAFE_STAT, f.data) != f.data[7]) return false;
  s->flags2 = f.data[0];
  s->ctrl_alive_echo = f.data[1] & 0x0F;
  s->inputs_fresh = f.data[1] >> 4;
  s->steer_track_err_deg = get_i16(&f.data[2]) * 0.01;
  s->lat_acc_meas = get_i16(&f.data[4]) * 0.01;
  s->alive = f.data[6] & 0x0F;
  return true;
}

}  // namespace vcan
}  // namespace bmpc_bridge

#endif

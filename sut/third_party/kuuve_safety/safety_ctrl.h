/* 안전 제어기(DUT) - 조향/가감속 명령 검사 계층.
 *
 * PC(제어)가 계산한 명령을 그대로 믿지 않고, 수신 무결성(CRC, Alive, 타임아웃)과
 * 범위, 변화율, 물리 제약을 검사해 안전한 값만 내보낸다. 하드웨어 의존성이 없는
 * C99 코드라서 PC(gcc/gcov)와 MCU(S32K312, 이후 AURIX)에서 같은 소스를 쓴다.
 * 시간은 호출자가 ms 단위로 넣어 준다(결정적 테스트를 위해 내부에서 시계를 읽지 않는다).
 *
 * 요구사항 매핑은 docs/tsr.md 를 본다.
 */
#ifndef SAFETY_CTRL_H
#define SAFETY_CTRL_H

#include <stdint.h>

typedef enum {
    SC_STATE_INIT = 0,
    SC_STATE_NORMAL = 1,
    SC_STATE_DEGRADED = 2,
    SC_STATE_SAFE_STOP = 3
} sc_state_t;

/* SAFE_OUT.FaultFlags (해당 주기에 활성인 비트) */
#define SC_FLAG_CRC_ERR       0x01u  /* TSR-04 */
#define SC_FLAG_ALIVE_FREEZE  0x02u  /* TSR-04 */
#define SC_FLAG_TIMEOUT       0x04u  /* TSR-03 */
#define SC_FLAG_STEER_CLAMP   0x08u  /* TSR-01 */
#define SC_FLAG_RATE_LIMIT    0x10u  /* TSR-02 */
#define SC_FLAG_LATACC_LIMIT  0x20u  /* TSR-05 */
#define SC_FLAG_SPEED_REJECT  0x40u  /* TSR-06 */
#define SC_FLAG_ACCEL_CLAMP   0x80u  /* TSR-01 (가감속 범위) */

/* E2E Data ID (TSR-07): CRC 계산에 메시지별 ID 를 먼저 넣어, 다른 메시지의 정상 프레임이
 * 이 메시지 ID 로 들어와도(위장, 오배달) CRC 불일치로 걸러낸다. CAN ID 와 일부러 다른 값을 쓴다. */
#define SC_DATA_ID_CTRL_CMD   0x0A01u
#define SC_DATA_ID_VEH_STATE  0x0A02u
#define SC_DATA_ID_SAFE_OUT   0x0A03u
#define SC_DATA_ID_SAFE_STAT  0x0A04u
/* 차량 CAN (dbc/vehicle.dbc): 센서 ECU 가 직접 내는 메시지. PC 가 전달하는 값과 독립된 출처 */
#define SC_DATA_ID_ESC_SPEED  0x0C01u
#define SC_DATA_ID_SAS_STEER  0x0C02u
#define SC_DATA_ID_VCU_PEDAL  0x0C03u
#define SC_DATA_ID_ACU_IMU    0x0C05u

/* SAFE_STAT.Flags2 (차량 입력 검사, 해당 주기에 활성인 비트) */
#define SC_FLAG2_SPEED_TIMEOUT   0x01u  /* TSR-09: 차속(VEH_STATE/ESC_SPEED) 두절 */
#define SC_FLAG2_DRIVER_OVERRIDE 0x02u  /* TSR-10: 운전자 브레이크 개입 */
#define SC_FLAG2_LATACC_MEAS     0x04u  /* TSR-11: 실측 횡가속도 초과 */
#define SC_FLAG2_SENSOR_MISMATCH 0x08u  /* TSR-12: ESC 와 IMU 횡가속도 불일치 */
#define SC_FLAG2_STEER_TRACK     0x10u  /* TSR-13: 실제 조향각이 출력을 따라오지 않음 */
#define SC_FLAG2_VEH_CRC         0x20u  /* 차량 CAN 프레임 CRC 불일치 (TSR-07 과 같은 E2E) */

/* SAFE_STAT.InputFresh: 차량 입력별 최근 수신 여부 */
#define SC_IN_SPEED  0x01u
#define SC_IN_IMU    0x02u
#define SC_IN_PEDAL  0x04u
#define SC_IN_SAS    0x08u

typedef struct {
    float    steer_limit_deg;        /* TSR-01: 조향각 한계 */
    float    steer_rate_dps;         /* TSR-02: 조향 변화율 한계 */
    uint32_t cmd_timeout_ms;         /* TSR-03: 명령 프레임 미수신 허용 시간 */
    uint8_t  alive_freeze_limit;     /* TSR-04: 이 횟수 연속 Alive 정지면 DEGRADED */
    uint8_t  init_valid_frames;      /* INIT -> NORMAL 에 필요한 연속 정상 프레임 수 */
    uint8_t  recover_valid_frames;   /* DEGRADED -> NORMAL 에 필요한 연속 정상 프레임 수 */
    float    accel_min_mps2;         /* TSR-01: 가감속 하한 */
    float    accel_max_mps2;         /* TSR-01: 가감속 상한 */
    float    safe_decel_mps2;        /* SAFE_STOP 에서 요구하는 감속도(음수) */
    float    wheelbase_m;            /* TSR-05: 횡가속도 계산용 축거 */
    float    lat_acc_max_mps2;       /* TSR-05: 횡가속도 한계 */
    float    speed_max_kph;          /* TSR-06: 속도 상한(초과하면 이상치) */
    float    speed_jump_max_kph;     /* TSR-06: 프레임 간 허용 속도 변화 */
    uint8_t  speed_reject_limit;     /* TSR-06: 이 횟수 연속 거부되면 실제 변화로 보고 수용 */
    uint32_t speed_timeout_ms;       /* TSR-09: 차속 미수신 허용 시간 (한 번이라도 받은 뒤) */
    uint32_t veh_input_timeout_ms;   /* IMU/페달/조향각 입력을 '최근' 으로 보는 시간. 지나면 해당 검사를 하지 않는다 */
    float    brake_override_pct;     /* TSR-10: 시스템이 감속하지 않는데 이 이상 브레이크 -> 운전자 개입, 자율 가속 금지 */
    float    lat_acc_meas_max_mps2;  /* TSR-11: 실측(IMU) 횡가속도 한계 */
    float    lat_acc_mismatch_mps2;  /* TSR-12: ESC 와 IMU 횡가속도 차이 한계 */
    uint8_t  mismatch_frames;        /* TSR-12: 이 횟수 연속 불일치면 고장 */
    uint8_t  steer_track_enable;     /* TSR-13: 조향각 센서 부호/배율 보정 전에는 0 (오차는 계속 보고) */
    float    sas_scale;              /* TSR-13: 조향각 센서 값 x 이 값 = 출력 조향(deg) 과 같은 기준 */
    float    steer_track_err_deg;    /* TSR-13: 허용 추종 오차 */
    uint32_t steer_track_ms;         /* TSR-13: 오차가 이 시간 넘게 이어지면 고장 */
} sc_config_t;

typedef struct {
    float      steer_deg;
    float      accel_mps2;
    sc_state_t state;
    uint8_t    flags;
    uint8_t    rx_age_10ms;          /* 마지막 유효 CTRL_CMD 이후 경과 시간 (10 ms 단위, 15 에서 포화) */
    uint8_t    bus_off_seen;         /* TSR-08: 리셋 이후 CAN 버스오프가 있었으면 1 */
    uint8_t    flags2;               /* SC_FLAG2_* (SAFE_STAT) */
    uint8_t    inputs_fresh;         /* SC_IN_* */
    uint8_t    ctrl_alive_echo;      /* 이 출력에 반영된 마지막 CTRL_CMD Alive (PC 가 왕복 지연을 잰다) */
    float      steer_track_err_deg;  /* 조향각 센서 x sas_scale - 직전 출력 조향 */
    float      lat_acc_meas_mps2;    /* 마지막 IMU 횡가속도 */
} sc_out_t;

typedef struct {
    sc_config_t cfg;
    sc_state_t  state;
    uint8_t     pending_flags;

    float       cmd_steer_deg;       /* 마지막으로 받아들인 명령 */
    float       cmd_accel_mps2;
    float       speed_kph;           /* 마지막으로 받아들인 속도 */
    uint8_t     speed_seen;
    uint8_t     speed_reject_run;

    uint8_t     alive_prev;
    uint8_t     alive_seen;
    uint8_t     stale_count;
    uint8_t     fresh_count;
    uint32_t    last_rx_ms;          /* CRC 가 맞는 마지막 CTRL_CMD 수신 시각 */

    float       out_steer_deg;
    uint32_t    last_step_ms;
    uint8_t     step_seen;
    uint8_t     bus_off_seen;

    /* 차량 CAN 입력 (TSR-09~13) */
    uint8_t     pending_flags2;
    uint8_t     speed_lost;          /* TSR-09: 차속 두절 중 (SAFE_STAT 입력 표시) */
    uint32_t    last_speed_ms;
    uint32_t    last_imu_ms;
    uint32_t    last_pedal_ms;
    uint32_t    last_sas_ms;
    uint32_t    last_esc_ms;
    uint8_t     imu_seen;
    uint8_t     pedal_seen;
    uint8_t     sas_seen;
    uint8_t     esc_lat_seen;
    float       esc_lat_mps2;
    float       imu_lat_mps2;
    float       brake_pct;
    float       sas_deg;
    uint8_t     mismatch_run;
    uint32_t    track_err_ms;
    float       out_accel_mps2;      /* 직전 출력 가감속 (TSR-10: 시스템 감속 중인지) */
} sc_t;

void sc_default_config(sc_config_t *cfg);
void sc_init(sc_t *sc, const sc_config_t *cfg);
/* SAFE_STOP 은 래치된다. 호출자가 원인 해소를 확인한 뒤에만 부른다. */
void sc_reset(sc_t *sc);

void sc_on_ctrl_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms);
void sc_on_veh_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms);
/* 차량 CAN (dbc/vehicle.dbc). ESC_SPEED 의 차속은 VEH_STATE 와 같은 검사(TSR-06)를 거쳐 같은 차속으로 쓴다 */
void sc_on_esc_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms);
void sc_on_imu_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms);
void sc_on_pedal_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms);
void sc_on_sas_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms);
/* TSR-08: CAN 컨트롤러가 버스오프에 들어갔을 때 호출한다. NORMAL/DEGRADED 이면 다음 주기에 SAFE_STOP. */
void sc_on_bus_off(sc_t *sc);
/* 제어 주기(10 ms)마다 호출한다. */
void sc_step(sc_t *sc, uint32_t now_ms, sc_out_t *out);

/* SAFE_OUT 프레임 조립 (Alive 는 호출자가 관리) */
void sc_pack_out(const sc_out_t *out, uint8_t alive, uint8_t data[8]);
/* SAFE_STAT 프레임 조립 (0x201: Flags2, CtrlAlive 에코, 입력 수신 여부, 조향 추종 오차, 실측 횡가속도) */
void sc_pack_stat(const sc_out_t *out, uint8_t alive, uint8_t data[8]);

/* CRC8 SAE-J1850 (poly 0x1D, init 0xFF, xorout 0xFF) */
uint8_t sc_crc8(const uint8_t *data, uint8_t len);
/* E2E CRC: Data ID 하위 바이트, 상위 바이트, 그다음 data[0..len-1] 순서로 CRC8 SAE-J1850 */
uint8_t sc_crc8_e2e(uint16_t data_id, const uint8_t *data, uint8_t len);
/* byte0..6 과 Data ID 로 CRC 를 계산해 byte7 에 넣는다 (PC 브리지와 테스트에서 사용) */
void sc_frame_seal(uint8_t data[8], uint16_t data_id);

#endif /* SAFETY_CTRL_H */

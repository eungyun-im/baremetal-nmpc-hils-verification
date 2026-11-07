/* S32K312EVB-Q172 HILS 펌웨어. SILS 가상 MCU(platform/linux/src/sim_mcu_main.c)와 같은 앱 코드(bmpc_app)를 돌리고,
 * 다른 점은 HAL 두 개뿐이다: 시각은 DWT 사이클 카운터, CAN 은 FlexCAN0 (500 kbps, J32).
 *
 *   수신 (인터럽트 -> 링 버퍼)  0x0F0 설정, 0x0F1 설정 확정, 0x100..0x105 요청 (메일박스 0..7, ID 마다 하나)
 *   메인 루프                   링 버퍼 -> bmpc_app_on_frame. 0x105 가 오면 그 자리에서 controller 한 주기를 돌리고
 *                               응답 0x1E0..0x1E3 을 보낸다 (계산 중에도 수신 인터럽트는 링 버퍼에 쌓는다)
 *   송신                        0x1E0..0x1E3 응답 (메일박스 8..11), 0x1E8 설정 ACK (12), 0x321 하트비트 (13)
 *   LED                         파랑 = 설정 전, 초록 = 동작, 빨강 = 최근 0.5 s 안에 오류 응답
 *
 * 하드웨어 설정(핀, 클럭, FlexCAN, 메시지 RAM ECC 초기화, 클럭 보정)은 팀 capstone_hils 의 fw_mpc_s32k312/main.c
 * (실보드 검증본)에서 가져왔다. 코어는 FIRC 48 MHz 이다 (그쪽 PLL 시도는 칩이 리셋되어 쓰지 않는다).
 *
 * 빌드 옵션 (build.sh 가 넘긴다)
 *   BMPC_FW_DCACHE        데이터 캐시 켬 (기본). SRAM·flash 상수 읽기에 쓰인다
 *   BMPC_FW_APP_SRAM      bmpc_app 을 SRAM 에 둔다 (비교용. 기본은 대기 없는 DTCM, float 약 38 KB)
 *   BMPC_FW_HEARTBEAT_MS  하트비트 주기 (기본 500, 0 = 끔). 브리지는 이 프레임을 stale_frames 로 센다
 *   BMPC_FW_PLL           코어 120 MHz (기본 켬, build.sh PLL=0 이면 FIRC 48 MHz)
 *   BMPC_FW_SAFETY        팀 안전 제어기 통합 (integ/bmpc_safety, build.sh SAFETY=1): MPC 응답을 CTRL_CMD 로 안전 제어기에 넣고
 *                         SysTick 10 ms 마다 SAFE_OUT 0x200 / SAFE_STAT 0x201 을 낸다. 차량 CAN 0x1A0..0x1D0, 0x7F0 은 수신 인터럽트에서
 *                         바로 안전 제어기로. LED 는 안전 상태 (초록 NORMAL, 파랑 DEGRADED, 빨강 SAFE_STOP, 꺼짐 INIT)
 *   BMPC_PROFILE          구간별 시간 (core 의 bmpc_prof.h). 하트비트마다 요청 1 회 평균을 10 us 단위로 보낸다
 *                         0x3F0: 요청 수 | build | 그중 전파 | 그중 비용 항,  0x3F1: 비선형 비용 | QP | 그중 Cholesky | 그중 기울기 */
#include <stdint.h>

#include "bmpc/bmpc_app.h"
#ifdef BMPC_FW_SAFETY
#include "bmpc_safety.h"
#endif
#include "bmpc/bmpc_math.h"
#include "bmpc/bmpc_prof.h"

#ifndef BMPC_FW_HEARTBEAT_MS
#define BMPC_FW_HEARTBEAT_MS 500u
#endif

#define REG32(a) (*(volatile uint32_t *)(a))
#define REG8(a) (*(volatile uint8_t *)(a))

#define CAN_TIMER_TICKS_10MS 5000u

/* ---- SWT_0 ---- */
#define SWT0_CR REG32(0x40270000u)
#define SWT0_TO REG32(0x40270008u)
#define SWT0_SR REG32(0x40270010u)
/* 워치독 제한 시간: 최악 풀이(약 45 ms)보다 넉넉히. SWT 클럭 SIRC 32 kHz */
#define SWT_TIMEOUT_TICKS 6400u /* 200 ms */

/* ---- SIUL2 ---- */
#define SIUL2_BASE 0x40290000u
#define MSCR(i) REG32(SIUL2_BASE + 0x240u + 4u * (i))
#define IMCR(i) REG32(SIUL2_BASE + 0xA40u + 4u * (i))
#define GPDO(i) REG8(SIUL2_BASE + 0x1300u + ((i) ^ 3u))
#define MSCR_OBE (1u << 21)
#define MSCR_IBE (1u << 19)
#define MSCR_SSS(x) ((x) & 0xFu)

#define PIN_PTA6 6u
#define PIN_PTA7 7u
#define PIN_PTA29 29u
#define PIN_PTA30 30u
#define PIN_PTA31 31u
#define PIN_PTC20 84u
#define PIN_PTC21 85u
#define LED_R PIN_PTA29
#define LED_G PIN_PTA30
#define LED_B PIN_PTA31

/* ---- MC_ME / MC_CGM ---- */
#define MC_ME_BASE 0x402DC000u
#define MC_ME_CTL_KEY REG32(MC_ME_BASE + 0x000u)
#define MC_ME_PRTN1_PCONF REG32(MC_ME_BASE + 0x300u)
#define MC_ME_PRTN1_PUPD REG32(MC_ME_BASE + 0x304u)
#define MC_ME_PRTN1_STAT REG32(MC_ME_BASE + 0x308u)
#define MC_ME_PRTN1_COFB2_CLKEN REG32(MC_ME_BASE + 0x338u)
#define FLEXCAN0_CLK_EN 0x2u
#define MC_CGM_BASE 0x402D8000u
#define MC_CGM_MUX3_DC0 REG32(MC_CGM_BASE + 0x3C8u)
#define MC_CGM_DC_DE (1u << 31)

/* ---- FlexCAN0 ---- */
#define CAN0_BASE 0x40304000u
#define CAN_MCR REG32(CAN0_BASE + 0x00u)
#define CAN_CTRL1 REG32(CAN0_BASE + 0x04u)
#define CAN_TIMER REG32(CAN0_BASE + 0x08u)
#define CAN_RXMGMASK REG32(CAN0_BASE + 0x10u)
#define CAN_RX14MASK REG32(CAN0_BASE + 0x14u)
#define CAN_RX15MASK REG32(CAN0_BASE + 0x18u)
#define CAN_ESR1 REG32(CAN0_BASE + 0x20u)
#define CAN_IMASK1 REG32(CAN0_BASE + 0x28u)
#define CAN_IFLAG1 REG32(CAN0_BASE + 0x30u)
#define CAN_CTRL2 REG32(CAN0_BASE + 0x34u)
#define CAN_RXFGMASK REG32(CAN0_BASE + 0x48u)
#define CAN_ERRSR REG32(CAN0_BASE + 0xAFCu)
#define CTRL2_WRMFRZ (1u << 28)
#define CAN0_RAM_CLEAR_WORDS (96u * 4u)
#define CAN0_RXIMR_COUNT 64u
#define CAN_MB_CS(n) REG32(CAN0_BASE + 0x80u + 16u * (n))
#define CAN_MB_ID(n) REG32(CAN0_BASE + 0x84u + 16u * (n))
#define CAN_MB_W0(n) REG32(CAN0_BASE + 0x88u + 16u * (n))
#define CAN_MB_W1(n) REG32(CAN0_BASE + 0x8Cu + 16u * (n))
#define MCR_MAXMB(x) ((x) & 0x7Fu)
#define MCR_SRXDIS (1u << 17)
#define MCR_FRZACK (1u << 24)
#define MCR_SOFTRST (1u << 25)
#define MCR_NOTRDY (1u << 27)
#define MCR_HALT (1u << 28)
#define MCR_FRZ (1u << 30)
#define MCR_MDIS (1u << 31)
#define MCR_LPMACK (1u << 20)
#define ESR1_BOFFINT (1u << 2)

#define CS_CODE(c) ((uint32_t)(c) << 24)
#define CS_CODE_OF(cs) (((cs) >> 24) & 0xFu)
#define CS_DLC(n) ((uint32_t)(n) << 16)
#define CODE_RX_EMPTY 0x4u
#define CODE_TX_INACTIVE 0x8u
#define CODE_TX_DATA 0xCu

/* 메일박스 배치: 수신 0..12 (ID 마다 하나, MB14/15 는 전용 마스크라 쓰지 않음), 송신 16..25 (한 ID 에 하나).
 * 수신 8..12 는 안전 제어기 입력 (차량 CAN, BENCH_RESET). 안전 제어기 없는 빌드는 받아서 버린다. */
#define NUM_MB 32u
#define N_RX_MB 13u
#define N_RX_MPC 8u
static const uint32_t k_rx_id[N_RX_MB] = {
    BMPC_ID_CFG_SET,       BMPC_ID_CFG_COMMIT,    BMPC_ID_REQ_BASE + 0u, BMPC_ID_REQ_BASE + 1u, BMPC_ID_REQ_BASE + 2u,
    BMPC_ID_REQ_BASE + 3u, BMPC_ID_REQ_BASE + 4u, BMPC_ID_REQ_BASE + 5u, 0x1A0u /* ESC_SPEED */, 0x1B0u /* SAS_STEER */,
    0x1C0u /* VCU_PEDAL */, 0x1D0u /* ACU_IMU */,  0x7F0u /* BENCH_RESET */};
#define MB_TX_RESP0 16u /* 0x1E0..0x1E3 -> 16..19 */
#define MB_TX_ACK 20u
#define MB_TX_HB 21u
#define MB_TX_PROF0 22u /* 0x3F0..0x3F3 (BMPC_PROFILE) -> 22, 23 */
#define MB_TX_SAFE_OUT 24u
#define MB_TX_SAFE_STAT 25u
#define MB_TX_SAFETY_DIAG 26u /* 0x3F4: 안전 주기 진단 (안전 제어기 빌드, 하트비트마다) */
#define ID_HEARTBEAT 0x321u /* capstone_hils 와 같은 ID·배치, 매직 'B''M' */

/* ---- SysTick / DWT / NVIC / SCB ---- */
#define SYST_CSR REG32(0xE000E010u)
#define SYST_RVR REG32(0xE000E014u)
#define SYST_CVR REG32(0xE000E018u)
#define DEMCR REG32(0xE000EDFCu)
#define DWT_CTRL REG32(0xE0001000u)
#define DWT_CYCCNT REG32(0xE0001004u)
#define DWT_LAR REG32(0xE0001FB0u)
#define NVIC_ISER(n) REG32(0xE000E100u + 4u * (n))
#define NVIC_IPR8(n) REG8(0xE000E400u + (n))
#define SCB_SHPR3 REG32(0xE000ED20u)
#define CAN0_MB_IRQ 110u
#define SCB_CCR REG32(0xE000ED14u)
#define SCB_CLIDR REG32(0xE000ED78u)
#define SCB_CCSIDR REG32(0xE000ED80u)
#define SCB_CSSELR REG32(0xE000ED84u)
#define SCB_ICIALLU REG32(0xE000EF50u)
#define SCB_DCISW REG32(0xE000EF60u)
#define CCR_DC (1u << 16)
#define CCR_IC (1u << 17)

/* 디버거로 읽는 진단값. DTCM 에 있어 캐시와 무관하게 보인다 */
volatile uint32_t g_dbg_cpm, g_dbg_dcache, g_dbg_requests, g_dbg_max_solve_us, g_dbg_max_total_us;
volatile uint32_t g_dbg_busoff, g_dbg_tx_overrun, g_dbg_id_mismatch;

static uint32_t g_cpm; /* 코어 사이클 / ms (부팅 때 FlexCAN 타이머로 잰다) */

static void led_set(uint32_t pin, uint32_t on) { GPDO(pin) = on ? 1u : 0u; }

static void swt_disable(void)
{
  SWT0_SR = 0xC520u;
  SWT0_SR = 0xD928u;
  SWT0_CR = SWT0_CR & ~1u;
}

#ifdef BMPC_FW_SWT
static void swt_enable(void)
{
  SWT0_SR = 0xC520u;
  SWT0_SR = 0xD928u;
  SWT0_TO = SWT_TIMEOUT_TICKS;
  SWT0_CR = SWT0_CR | 1u;
}
static void swt_service(void)
{
  SWT0_SR = 0xA602u;
  SWT0_SR = 0xB480u;
}
#endif

static void pins_init(void)
{
  MSCR(PIN_PTA7) = MSCR_OBE | MSCR_SSS(4); /* CAN0_TX */
  MSCR(PIN_PTA6) = MSCR_IBE;               /* CAN0_RX */
  IMCR(0) = 2u;
  MSCR(PIN_PTC21) = MSCR_OBE; /* TJA1153 EN, STB */
  MSCR(PIN_PTC20) = MSCR_OBE;
  GPDO(PIN_PTC21) = 1u;
  GPDO(PIN_PTC20) = 1u;
  MSCR(LED_R) = MSCR_OBE;
  MSCR(LED_G) = MSCR_OBE;
  MSCR(LED_B) = MSCR_OBE;
  led_set(LED_R, 0);
  led_set(LED_G, 0);
  led_set(LED_B, 0);
}

static void me_update(void)
{
  MC_ME_PRTN1_PUPD = 1u;
  MC_ME_CTL_KEY = 0x5AF0u;
  MC_ME_CTL_KEY = 0xA50Fu;
  while (MC_ME_PRTN1_PUPD & 1u) {
  }
}

static void clock_init(void)
{
  MC_ME_PRTN1_PCONF |= 1u;
  me_update();
  while (!(MC_ME_PRTN1_STAT & 1u)) {
  }
  MC_ME_PRTN1_COFB2_CLKEN |= FLEXCAN0_CLK_EN;
  me_update();
  MC_CGM_MUX3_DC0 = MC_CGM_DC_DE; /* FLEXCANA_CLK = FIRC 48 MHz / 1 */
}

/* ---- 코어 클럭 120 MHz (BMPC_FW_PLL, 기본 켬) ----
 * FXOSC 16 MHz -> PLL VCO 960 MHz (RDIV 2, MFI 120) -> ODIV2 /2 = 480 MHz -> PHI0 /4 = 120 MHz -> 코어 (MUX_0 선택 8).
 * 주변 분주 AIPS_PLAT /2 (60), AIPS_SLOW /4 (30), HSE /2 (60), DCM /4 (30). 플래시 읽기 대기 3 (133 MHz 이하).
 * 순서와 값은 NXP RTD Clock_Ip (Zephyr hal_nxp s32k3: Clock_Ip_Specific.c, Clock_Ip_Pll.c, S32K344 예제 설정)를 따랐다.
 * 팀 capstone_hils 시도(PLL 레지스터 접근에서 리셋)에는 MC_ME 의 FXOSC(REQ53)·PLL(REQ56) 블록 클럭 켜기가 없었다.
 * FlexCAN 은 FIRC(MUX_3)를 그대로 쓰므로 비트 타이밍은 바뀌지 않는다. 코어 MHz 는 부팅 때 FlexCAN 타이머로 다시 잰다.
 * 어느 단계든 시간 안에 안 끝나면 FIRC 48 MHz 로 남고 g_dbg_pll 에 단계를 남긴다 (0x80 | 단계). 성공하면 0x7F. */
#define MC_ME_PRTN1_COFB1_STAT REG32(MC_ME_BASE + 0x314u)
#define MC_ME_PRTN1_COFB1_CLKEN REG32(MC_ME_BASE + 0x334u)
#define ME_REQ53_FXOSC (1u << 21)
#define ME_REQ56_PLL (1u << 24)
#define FXOSC_CTRL REG32(0x402D4000u)
#define FXOSC_STAT REG32(0x402D4004u)
#define PLL_BASE 0x402E0000u
#define PLL_PLLCR REG32(PLL_BASE + 0x00u)
#define PLL_PLLSR REG32(PLL_BASE + 0x04u)
#define PLL_PLLDV REG32(PLL_BASE + 0x08u)
#define PLL_PLLFM REG32(PLL_BASE + 0x0Cu)
#define PLL_PLLFD REG32(PLL_BASE + 0x10u)
#define PLL_PLLCAL2 REG32(PLL_BASE + 0x18u)
#define PLL_PLLODIV(i) REG32(PLL_BASE + 0x80u + 4u * (i))
#define FLASH_CTL REG32(0x402EC00Cu)
#define CGM_MUX0_CSC REG32(MC_CGM_BASE + 0x300u)
#define CGM_MUX0_CSS REG32(MC_CGM_BASE + 0x304u)
#define CGM_MUX0_DC(n) REG32(MC_CGM_BASE + 0x308u + 4u * (n))
#define CGM_MUX0_DIV_UPD_STAT REG32(MC_CGM_BASE + 0x33Cu)

volatile uint32_t g_dbg_pll;

static int wait_bits(volatile uint32_t *r, uint32_t mask, uint32_t want)
{
  uint32_t n;
  for (n = 0; n < 4000000u; n++)
    if ((*r & mask) == want) return 1;
  return 0;
}

/* 플래시 읽기 대기 상태는 플래시에서 명령을 가져오는 중에 바꾸지 않는다 (RTD 와 같이 RAM 에서 실행). */
__attribute__((section(".ramfunc"), noinline, long_call)) static void flash_wait_states(uint32_t rwsc)
{
  uint32_t v = FLASH_CTL;
  v &= ~((1u << 15) | (0x1Fu << 8)); /* RWSL, RWSC */
  v |= (rwsc & 0x1Fu) << 8;
  __asm volatile("dsb\n isb");
  FLASH_CTL &= ~(1u << 15); /* 잠금 풀기 */
  FLASH_CTL = v;
  __asm volatile("dsb\n isb");
}

static void cgm_div(uint32_t dc, uint32_t div)
{
  uint32_t v = CGM_MUX0_DC(dc);
  v &= ~(7u << 16);
  v |= ((div - 1u) & 7u) << 16;
  CGM_MUX0_DC(dc) = v;
  (void)wait_bits(&CGM_MUX0_DIV_UPD_STAT, 1u, 0u);
  CGM_MUX0_DC(dc) |= MC_CGM_DC_DE;
}

static void pll_up(void)
{
  g_dbg_pll = 0x81;
  /* 1) FXOSC, PLL 블록 클럭 (파티션 1) */
  MC_ME_PRTN1_COFB1_CLKEN |= ME_REQ53_FXOSC | ME_REQ56_PLL;
  MC_ME_PRTN1_PCONF |= 1u;
  me_update();
  if (!wait_bits(&MC_ME_PRTN1_COFB1_STAT, ME_REQ53_FXOSC | ME_REQ56_PLL, ME_REQ53_FXOSC | ME_REQ56_PLL)) return;
  /* 2) FXOSC 16 MHz 크리스털: OSCON, EOCV 157, GM_SEL 12, COMP_EN (RTD 예제 값) */
  g_dbg_pll = 0x82;
  FXOSC_CTRL = (1u << 0) | (157u << 16) | (12u << 4) | (1u << 24);
  if (!wait_bits(&FXOSC_STAT, 0x80000000u, 0x80000000u)) return;
  /* 3) PLL: 끈 상태에서 분주·배수 설정 -> 켜고 잠김 대기 -> PHI0 출력 켜기 */
  g_dbg_pll = 0x83;
  PLL_PLLODIV(0) &= ~0x80000000u;
  PLL_PLLODIV(1) &= ~0x80000000u;
  PLL_PLLCR |= 0x80000000u;
  PLL_PLLDV = (PLL_PLLDV & ~(0x7E000000u | 0x7000u | 0xFFu)) | (2u << 25) | (2u << 12) | 120u; /* ODIV2 2, RDIV 2, MFI 120 */
  PLL_PLLFD &= ~(0x7FFFu | 0x40000000u);   /* MFN 0, SDMEN 0 */
  PLL_PLLFM = 0x40000000u;                 /* SSCGBYP: 주파수 변조 없음 */
  PLL_PLLCAL2 = 0u;                        /* ULKCTL (변조 없을 때 권장 0) */
  PLL_PLLODIV(0) = 0x80000000u | (3u << 16); /* PHI0 = 480 / 4 = 120 MHz */
  PLL_PLLCR &= ~0x80000000u;
  g_dbg_pll = 0x84;
  if (!wait_bits(&PLL_PLLSR, 0x4u, 0x4u)) return;
  /* 4) 플래시 대기 상태 -> 주변 분주 -> 코어 mux 를 PHI0 로 */
  g_dbg_pll = 0x85;
  flash_wait_states(3u);
  cgm_div(1u, 2u); /* AIPS_PLAT */
  cgm_div(2u, 4u); /* AIPS_SLOW */
  cgm_div(3u, 2u); /* HSE */
  cgm_div(4u, 4u); /* DCM */
  cgm_div(0u, 1u); /* CORE */
  g_dbg_pll = 0x86;
  if (!wait_bits(&CGM_MUX0_CSS, 1u << 16, 0u)) return; /* SWIP 없음 */
  CGM_MUX0_CSC = (CGM_MUX0_CSC & ~0x0F000000u) | (8u << 24) | (1u << 2); /* SELCTL = PLL_PHI0, CLK_SW */
  g_dbg_pll = 0x87;
  if (!wait_bits(&CGM_MUX0_CSS, 1u << 16, 0u)) return;
  if (((CGM_MUX0_CSS >> 24) & 0xFu) != 8u) return;
  g_dbg_pll = 0x7F;
}

static void icache_enable(void)
{
  __asm volatile("dsb\n isb");
  SCB_ICIALLU = 0u;
  __asm volatile("dsb\n isb");
  SCB_CCR |= CCR_IC;
  __asm volatile("dsb\n isb");
}

#ifdef BMPC_FW_DCACHE
/* L1 데이터 캐시: 무효화(set/way) 뒤 켠다 (CMSIS SCB_EnableDCache 와 같은 순서).
 * MPU 를 켜지 않으므로 기본 메모리 맵을 따른다: SRAM(0x2040_0000) 은 write-back 캐시, 주변장치는 Device(캐시 안 함),
 * DTCM 은 캐시를 거치지 않는다. DMA 를 쓰지 않으니 일관성 문제는 없다. 캐시가 없는 칩이면 아무것도 안 하고 0 을 돌려준다. */
static uint32_t dcache_enable(void)
{
  uint32_t ccsidr, sets, ways;

  if ((SCB_CLIDR & 2u) == 0u) return 0u; /* Ctype1: 데이터 캐시 없음 */
  SCB_CSSELR = 0u;                       /* L1 데이터 캐시 선택 */
  __asm volatile("dsb");
  ccsidr = SCB_CCSIDR;
  sets = (ccsidr >> 13) & 0x7FFFu;
  do {
    ways = (ccsidr >> 3) & 0x3FFu;
    do {
      SCB_DCISW = ((sets << 5) & 0x3FE0u) | ((ways << 30) & 0xC0000000u);
    } while (ways-- != 0u);
  } while (sets-- != 0u);
  __asm volatile("dsb");
  SCB_CCR |= CCR_DC;
  __asm volatile("dsb\n isb");
  return 1u;
}
#endif

static void can_arm_rx(uint32_t mb, uint32_t id)
{
  CAN_MB_ID(mb) = id << 18;
  CAN_MB_CS(mb) = CS_CODE(CODE_RX_EMPTY);
}

static void can_init(void)
{
  uint32_t i;

  CAN_MCR &= ~MCR_MDIS;
  while (CAN_MCR & MCR_LPMACK) {
  }
  CAN_MCR |= MCR_SOFTRST;
  while (CAN_MCR & MCR_SOFTRST) {
  }
  CAN_MCR |= MCR_FRZ | MCR_HALT;
  while (!(CAN_MCR & MCR_FRZACK)) {
  }

  CAN_CTRL1 = (5u << 24) | (1u << 22) | (3u << 19) | (3u << 16) | 6u; /* 500 kbps, 샘플 75% */
  CAN_MCR = (CAN_MCR & ~0x7Fu) | MCR_MAXMB(NUM_MB - 1u) | MCR_SRXDIS;

  /* 메시지 RAM 은 ECC 보호라 전체(0x80~0x67F, RXIMR, 마스크)를 0 으로 써야 전원 재투입 후 Freeze 되지 않는다 */
  CAN_CTRL2 |= CTRL2_WRMFRZ;
  for (i = 0; i < CAN0_RAM_CLEAR_WORDS; i++) REG32(CAN0_BASE + 0x80u + 4u * i) = 0;
  for (i = 0; i < CAN0_RXIMR_COUNT; i++) REG32(CAN0_BASE + 0x880u + 4u * i) = 0;
  CAN_RXMGMASK = 0;
  CAN_RX14MASK = 0;
  CAN_RX15MASK = 0;
  CAN_RXFGMASK = 0;
  CAN_CTRL2 &= ~CTRL2_WRMFRZ;
  CAN_ERRSR = 0x000D000Du;

  CAN_RXMGMASK = 0x1FFC0000u; /* 표준 ID 11 비트 전부 비교 */
  CAN_IFLAG1 = 0xFFFFFFFFu;
  CAN_ESR1 = 0xFFFFFFFFu;
  for (i = 0; i < N_RX_MB; i++) can_arm_rx(i, k_rx_id[i]);
  CAN_IMASK1 = (1u << N_RX_MB) - 1u; /* 수신 메일박스만 인터럽트 */

  CAN_MCR &= ~(MCR_HALT | MCR_FRZ);
  while (CAN_MCR & (MCR_FRZACK | MCR_NOTRDY)) {
  }
}

/* 앞 프레임이 아직 버스에 못 나갔으면 잠깐(약 2 ms) 기다린다. 그래도 남아 있으면 덮어쓰고 g_dbg_tx_overrun 을 센다. */
static int can_send(uint32_t mb, uint32_t id, const uint8_t *d, uint32_t len)
{
  uint32_t w0 = 0, w1 = 0, i;
  const uint32_t t0 = DWT_CYCCNT;
  int rc = 0;

  while (CS_CODE_OF(CAN_MB_CS(mb)) == CODE_TX_DATA) {
    if (DWT_CYCCNT - t0 > 2u * g_cpm) {
      ++g_dbg_tx_overrun;
      rc = -1;
      break;
    }
  }
  if (len > 8u) len = 8u;
  for (i = 0; i < len; i++) {
    if (i < 4u) w0 |= (uint32_t)d[i] << (24u - 8u * i);
    else w1 |= (uint32_t)d[i] << (24u - 8u * (i - 4u));
  }
  CAN_IFLAG1 = 1u << mb;
  CAN_MB_CS(mb) = CS_CODE(CODE_TX_INACTIVE);
  CAN_MB_ID(mb) = id << 18;
  CAN_MB_W0(mb) = w0;
  CAN_MB_W1(mb) = w1;
  CAN_MB_CS(mb) = CS_CODE(CODE_TX_DATA) | CS_DLC(len);
  return rc;
}

#ifdef BMPC_FW_SAFETY
static bs_app_t g_bs;                 /* MPC + 안전 제어기 (DTCM) */
static volatile uint32_t g_tick_ms;   /* SysTick ms (안전 제어기 시간) */

/* SysTick 1 ms: 10 ms 마다 안전 제어기 한 주기 -> SAFE_OUT, SAFE_STAT. MPC 계산 중에도 멈추지 않는다 */
/* 안전 주기 진단: sc_step 사이 간격 최대, 한 번 실행 시간 최대 (DWT, 하트비트마다 비운다), 누적 횟수 */
static volatile uint32_t g_safe_steps, g_safe_gap_max_cyc, g_safe_exec_max_cyc, g_safe_last_cyc;

void SysTick_Handler(void);
void SysTick_Handler(void)
{
  const uint32_t t = ++g_tick_ms;
  if (t % BS_PERIOD_MS == 0u) {
    const uint32_t c0 = DWT_CYCCNT;
    if (g_safe_steps > 0u && c0 - g_safe_last_cyc > g_safe_gap_max_cyc) g_safe_gap_max_cyc = c0 - g_safe_last_cyc;
    g_safe_last_cyc = c0;
    bs_safety_tick(&g_bs, t);
    if (DWT_CYCCNT - c0 > g_safe_exec_max_cyc) g_safe_exec_max_cyc = DWT_CYCCNT - c0;
    ++g_safe_steps;
  }
}
#endif

/* ---- 수신 링 버퍼 (ISR 생산, 메인 루프 소비) ---- */
#define RX_RING 128u
typedef struct {
  uint16_t id;
  uint8_t dlc;
  uint8_t d[8];
} rx_frame_t;
static volatile rx_frame_t g_ring[RX_RING];
static volatile uint32_t g_head, g_tail;
static volatile uint32_t g_rx_overflow;

void CAN0_MB_IRQHandler(void);
void CAN0_MB_IRQHandler(void)
{
  const uint32_t flags = CAN_IFLAG1 & ((1u << N_RX_MB) - 1u);
  uint32_t mb;

  for (mb = 0; mb < N_RX_MB; mb++) {
    uint32_t cs, rid, w0, w1, i, h;
    if (!(flags & (1u << mb))) continue;
    cs = CAN_MB_CS(mb);
    rid = (CAN_MB_ID(mb) >> 18) & 0x7FFu;
    w0 = CAN_MB_W0(mb);
    w1 = CAN_MB_W1(mb);
    (void)CAN_TIMER; /* 메일박스 잠금 해제 */
    CAN_IFLAG1 = 1u << mb;
    can_arm_rx(mb, k_rx_id[mb]);
    if (rid != k_rx_id[mb]) {
      ++g_dbg_id_mismatch;
      continue;
    }
    if (mb >= N_RX_MPC) {
#ifdef BMPC_FW_SAFETY
      /* 안전 제어기 입력은 MPC 계산을 기다리지 않고 바로 (수신 시각이 차속 두절 판정 TSR-09 에 쓰인다) */
      bmpc_can_frame_t f;
      f.id = rid;
      f.dlc = (uint8_t)((cs >> 16) & 0xFu);
      for (i = 0; i < 8u; i++) f.data[i] = (i < 4u) ? (uint8_t)(w0 >> (24u - 8u * i)) : (uint8_t)(w1 >> (24u - 8u * (i - 4u)));
      bs_on_safety_frame(&g_bs, &f, g_tick_ms);
#endif
      continue;
    }
    h = g_head;
    if (((h + 1u) % RX_RING) == g_tail) {
      ++g_rx_overflow;
      continue;
    }
    g_ring[h].id = (uint16_t)rid;
    g_ring[h].dlc = (uint8_t)((cs >> 16) & 0xFu);
    for (i = 0; i < 8u; i++) {
      g_ring[h].d[i] = (i < 4u) ? (uint8_t)(w0 >> (24u - 8u * i)) : (uint8_t)(w1 >> (24u - 8u * (i - 4u)));
    }
    g_head = (h + 1u) % RX_RING;
  }
}

/* ---- 시각 ---- */
static uint64_t g_ms;
static uint32_t g_cyc_acc, g_cyc_last;

/* SysTick 과 CAN 타이머(비트 시간 = 2 us)로 10 ms 동안의 코어 사이클을 센다. 코어 클럭이 부트 경로마다 달라서 가정하지 않는다. */
static uint32_t calibrate_cycles_per_ms(void)
{
  uint16_t t0;
  uint32_t c0, c1;

  SYST_RVR = 0x00FFFFFFu;
  SYST_CVR = 0;
  SYST_CSR = 5u;
  t0 = (uint16_t)CAN_TIMER;
  while ((uint16_t)((uint16_t)CAN_TIMER - t0) == 0u) {
  }
  t0 = (uint16_t)CAN_TIMER;
  c0 = SYST_CVR & 0x00FFFFFFu;
  while ((uint16_t)((uint16_t)CAN_TIMER - t0) < CAN_TIMER_TICKS_10MS) {
  }
  c1 = SYST_CVR & 0x00FFFFFFu;
  SYST_CSR = 0;
  return ((c0 - c1) & 0x00FFFFFFu) / 10u;
}

/* DWT 를 누적해 ms 를 센다. 부르는 간격이 2^32 사이클(48 MHz 에서 89 s)보다 짧으면 정확하다. 메인 문맥에서만 부른다. */
static void clock_advance(void)
{
  const uint32_t c = DWT_CYCCNT;
  g_cyc_acc += c - g_cyc_last;
  g_cyc_last = c;
  if (g_cyc_acc >= g_cpm) {
    g_ms += g_cyc_acc / g_cpm;
    g_cyc_acc %= g_cpm;
  }
}

static uint64_t now_ms(void)
{
  clock_advance();
  return g_ms;
}

/* HAL: 단조 증가 us (부팅 기준) */
static int64_t hal_now_us(void *ctx)
{
  (void)ctx;
  clock_advance();
  return (int64_t)g_ms * 1000 + (int64_t)((g_cyc_acc * 1000u) / g_cpm);
}

/* HAL: CAN 프레임 1 개. ID 마다 송신 메일박스를 따로 써서 응답 4 프레임이 서로 덮어쓰지 않는다 */
static int hal_can_send(void *ctx, const bmpc_can_frame_t *f)
{
  uint32_t mb = MB_TX_HB;
  (void)ctx;
  if (f->id >= BMPC_ID_RESP_BASE && f->id < BMPC_ID_RESP_BASE + BMPC_RESP_FRAMES) mb = MB_TX_RESP0 + (f->id - BMPC_ID_RESP_BASE);
  else if (f->id == BMPC_ID_CFG_ACK) mb = MB_TX_ACK;
  else if (f->id == 0x200u) mb = MB_TX_SAFE_OUT; /* SAFE_OUT, SAFE_STAT: SysTick 인터럽트에서만 보낸다 */
  else if (f->id == 0x201u) mb = MB_TX_SAFE_STAT;
  return can_send(mb, f->id, f->data, f->dlc);
}

#ifdef BMPC_PROFILE
volatile uint32_t bmpc_prof[BMPC_PROF_COUNT];
uint32_t bmpc_prof_now(void) { return DWT_CYCCNT; }

/* 지난 전송 뒤 요청 1 회 평균 (10 us 단위, u16) 을 보내고 누적을 비운다 */
static uint16_t prof_avg(uint32_t idx, uint32_t n)
{
  const uint64_t v = n ? ((uint64_t)bmpc_prof[idx] * 100u) / ((uint64_t)g_cpm * n) : 0u;
  return (uint16_t)(v > 0xFFFFu ? 0xFFFFu : v);
}

static void put16(uint8_t *d, uint16_t v)
{
  d[0] = (uint8_t)v;
  d[1] = (uint8_t)(v >> 8);
}

/* 부팅 때 한 번: 같은 내적(256 개, DTCM)을 명령 캐시 켬/끔으로 재고, 캐시 레지스터를 읽어 둔다 (0x3F2, 0x3F3) */
static uint32_t g_bench[4]; /* 내적 사이클 (I 캐시 켬), (끔), SCB_CCR, SCB_CLIDR */
static bmpc_real g_ba[256], g_bb[256];
static volatile bmpc_real g_bsink;

static uint32_t bench_dot(void)
{
  uint32_t t0, k;
  t0 = DWT_CYCCNT;
  for (k = 0; k < 4u; ++k) g_bsink = bm_dot(g_ba, g_bb, 256);
  return (DWT_CYCCNT - t0) / 4u;
}

static void bench_boot(void)
{
  uint32_t k;
  for (k = 0; k < 256u; ++k) {
    g_ba[k] = (bmpc_real)k * (bmpc_real)0.001;
    g_bb[k] = (bmpc_real)1.0 - (bmpc_real)k * (bmpc_real)0.002;
  }
  (void)bench_dot();
  g_bench[0] = bench_dot();
  SCB_CCR &= ~CCR_IC;
  __asm volatile("dsb\n isb");
  g_bench[1] = bench_dot();
  icache_enable();
  g_bench[2] = SCB_CCR;
  g_bench[3] = SCB_CLIDR;
}

static void put32(uint8_t *d, uint32_t v)
{
  d[0] = (uint8_t)v;
  d[1] = (uint8_t)(v >> 8);
  d[2] = (uint8_t)(v >> 16);
  d[3] = (uint8_t)(v >> 24);
}

static void prof_send(uint32_t requests)
{
  static uint32_t last_requests;
  const uint32_t n = requests - last_requests;
  uint8_t d[8];
  uint32_t k;
  last_requests = requests;
  put16(&d[0], (uint16_t)(n > 0xFFFFu ? 0xFFFFu : n));
  put16(&d[2], prof_avg(BMPC_PROF_BUILD, n));
  put16(&d[4], prof_avg(BMPC_PROF_PROP, n));
  put16(&d[6], prof_avg(BMPC_PROF_TERM, n));
  (void)can_send(MB_TX_PROF0, 0x3F0u, d, 8u);
  put16(&d[0], prof_avg(BMPC_PROF_COST, n));
  put16(&d[2], prof_avg(BMPC_PROF_QP, n));
  put16(&d[4], prof_avg(BMPC_PROF_CHOL, n));
  put16(&d[6], prof_avg(BMPC_PROF_GRAD, n));
  (void)can_send(MB_TX_PROF0 + 1u, 0x3F1u, d, 8u);
  for (k = 0; k < (uint32_t)BMPC_PROF_COUNT; ++k) bmpc_prof[k] = 0u;
  put32(&d[0], g_bench[0]);
  put32(&d[4], g_bench[1]);
  (void)can_send(MB_TX_PROF0, 0x3F2u, d, 8u);
  put32(&d[0], g_bench[2]);
  put32(&d[4], g_bench[3]);
  (void)can_send(MB_TX_PROF0 + 1u, 0x3F3u, d, 8u);
}
#endif

/* controller 작업 메모리 (float 약 50 KB). 기본은 대기 없는 DTCM. SRAM 에 두면 같은 계산이 몇 배 느리다 */
#if defined(BMPC_FW_SAFETY)
#define APP (g_bs.app)
#elif defined(BMPC_FW_APP_SRAM)
static bmpc_app_t g_app __attribute__((section(".bss.sram.app")));
#define APP g_app
#else
static bmpc_app_t g_app;
#define APP g_app
#endif

void startup_c(void);
int main(void);
extern uint32_t __data_start, __data_end, __data_load, __bss_start, __bss_end;

extern uint32_t __ramfunc_start, __ramfunc_end, __ramfunc_load;

void startup_c(void)
{
  uint32_t *s = &__data_load, *d = &__data_start;
  {
    uint32_t *rs = &__ramfunc_load, *rd = &__ramfunc_start; /* RAM 에서 실행할 함수 (데이터 캐시 켜기 전에 복사) */
    while (rd < &__ramfunc_end) *rd++ = *rs++;
  }

  while (d < &__data_end) *d++ = *s++;
  for (d = &__bss_start; d < &__bss_end; d++) *d = 0;
  main();
  for (;;) {
  }
}

int main(void)
{
  bmpc_hal_t hal;
  uint8_t hb[8] = {'B', 'M', 0, 0, 0, 0, 0, 0};
  uint16_t hb_count = 0;
  uint64_t ms, next_hb = 500u, last_err_ms = 0;
  uint32_t last_errs = 0, have_err = 0;

  swt_disable();
  pins_init();
  clock_init();
#ifdef BMPC_FW_PLL
  {
    /* PLL 설정이 잘못돼 리셋이 반복돼도 디버거가 붙어 다시 플래시할 수 있게 1 초 기다린다 (FIRC 48 MHz) */
    volatile uint32_t n;
    for (n = 0; n < 12000000u; n++) {
    }
  }
  pll_up();
#endif
  can_init();
  icache_enable();
#ifdef BMPC_FW_DCACHE
  g_dbg_dcache = dcache_enable();
#endif

  g_cpm = calibrate_cycles_per_ms();
  g_dbg_cpm = g_cpm;
  DEMCR |= 1u << 24;
  DWT_LAR = 0xC5ACCE55u;
  DWT_CYCCNT = 0;
  DWT_CTRL |= 1u;
  g_cyc_last = DWT_CYCCNT;
#ifdef BMPC_PROFILE
  bench_boot();
#endif

  hal.now_us = hal_now_us;
  hal.can_send = hal_can_send;
  hal.ctx = 0;
#ifdef BMPC_FW_SAFETY
  {
    bs_cfg_t bcfg;
    sc_config_t scfg;
    bs_default_cfg(&bcfg);
    bs_default_safety_config(&scfg);
    bs_init(&g_bs, &bcfg, &scfg, &hal);
  }
  /* SysTick 1 ms, CAN 수신과 같은 우선순위 (서로 끼어들지 않는다) */
  SCB_SHPR3 = (SCB_SHPR3 & 0x00FFFFFFu) | (0x80u << 24);
  NVIC_IPR8(CAN0_MB_IRQ) = 0x80u;
  SYST_RVR = g_cpm - 1u;
  SYST_CVR = 0;
  SYST_CSR = 7u; /* 코어 클럭, 인터럽트, 켬 */
#else
  bmpc_app_init(&g_app, &hal);
#endif

  NVIC_ISER(CAN0_MB_IRQ / 32u) = 1u << (CAN0_MB_IRQ % 32u);
  __asm volatile("cpsie i");

#ifdef BMPC_FW_SWT
  swt_enable();
#endif
  for (;;) {
#ifdef BMPC_FW_SWT
    swt_service();
#ifdef BS_TEST_HOOK
    if (g_bs.hook_pending && g_bs.hook_kind == 3u) { /* 시험대: 메인 루프를 멈춰 워치독 리셋을 확인한다 */
      for (;;) {
      }
    }
#endif
#endif
    uint32_t errs;

    while (g_tail != g_head) {
      const uint32_t t = g_tail;
      bmpc_can_frame_t f;
      uint32_t i;
      f.id = g_ring[t].id;
      f.dlc = g_ring[t].dlc;
      for (i = 0; i < 8u; i++) f.data[i] = g_ring[t].d[i];
      g_tail = (t + 1u) % RX_RING;
#ifdef BMPC_FW_SAFETY
      bs_on_mpc_frame(&g_bs, &f);
      if (g_bs.ctrl_pending) {
        __asm volatile("cpsid i");
        (void)bs_feed_ctrl(&g_bs, g_tick_ms);
        __asm volatile("cpsie i");
      }
#else
      bmpc_app_on_frame(&g_app, &f);
#endif
    }
#ifdef BMPC_FW_SAFETY
    bs_note_applied(&g_bs, hal_now_us(0));
#endif
    if (CAN_ESR1 & ESR1_BOFFINT) { /* 버스오프는 FlexCAN 이 스스로 복구한다 (CTRL1.BOFFREC = 0). 세기만 한다 */
      CAN_ESR1 = ESR1_BOFFINT;
      ++g_dbg_busoff;
#ifdef BMPC_FW_SAFETY
      __asm volatile("cpsid i");
      bs_on_bus_off(&g_bs); /* TSR-08 */
      __asm volatile("cpsie i");
#endif
    }

    ms = now_ms();
    errs = APP.stats.rx_errors + APP.stats.solver_errors;
    if (errs != last_errs) {
      last_errs = errs;
      last_err_ms = ms;
      have_err = 1u;
    }
    if (have_err && ms - last_err_ms >= 500u) have_err = 0u;
#ifdef BMPC_FW_SAFETY
    led_set(LED_G, g_bs.out.state == SC_STATE_NORMAL);
    led_set(LED_B, g_bs.out.state == SC_STATE_DEGRADED);
    led_set(LED_R, g_bs.out.state == SC_STATE_SAFE_STOP);
    (void)have_err;
#else
    led_set(LED_B, !APP.ctrl.configured);
    led_set(LED_G, APP.ctrl.configured && !have_err);
    led_set(LED_R, have_err);
#endif

    g_dbg_requests = APP.stats.requests;
    g_dbg_max_solve_us = APP.stats.max_solve_us;
    g_dbg_max_total_us = APP.stats.max_total_us;

    /* 하트비트 (capstone_hils 의 read_heartbeat.py 와 같은 배치):
     * 'B' 'M' | 횟수 u16 | 수신 링 넘침 u16 | 코어 MHz | 최대 계산 ms (255 에서 멈춤) */
    if (BMPC_FW_HEARTBEAT_MS > 0u && ms >= next_hb) {
      const uint32_t solve_ms = APP.stats.max_solve_us / 1000u;
      next_hb = ms + BMPC_FW_HEARTBEAT_MS;
      hb[2] = (uint8_t)(hb_count >> 8);
      hb[3] = (uint8_t)hb_count;
      hb[4] = (uint8_t)(g_rx_overflow >> 8);
      hb[5] = (uint8_t)g_rx_overflow;
      hb[6] = (uint8_t)((g_cpm + 500u) / 1000u);
      hb[7] = (uint8_t)(solve_ms > 255u ? 255u : solve_ms);
      ++hb_count;
      (void)can_send(MB_TX_HB, ID_HEARTBEAT, hb, 8u);
#ifdef BMPC_FW_SAFETY
      {
        /* 0x3F4: 안전 주기 횟수 u32 | 지난 0.5 s 동안 sc_step 간격 최대 (10 us) u16 | 실행 시간 최대 (us) u8 | 버스오프 횟수 u8 */
        uint8_t sd[8];
        uint32_t steps, gap, exec;
        __asm volatile("cpsid i");
        steps = g_safe_steps;
        gap = g_safe_gap_max_cyc;
        exec = g_safe_exec_max_cyc;
        g_safe_gap_max_cyc = 0u;
        g_safe_exec_max_cyc = 0u;
        __asm volatile("cpsie i");
        gap = (uint32_t)(((uint64_t)gap * 100u) / g_cpm); /* 10 us 단위 */
        exec = (uint32_t)(((uint64_t)exec * 1000u) / g_cpm);
        sd[0] = (uint8_t)steps;
        sd[1] = (uint8_t)(steps >> 8);
        sd[2] = (uint8_t)(steps >> 16);
        sd[3] = (uint8_t)(steps >> 24);
        sd[4] = (uint8_t)(gap > 0xFFFFu ? 0xFFu : gap);
        sd[5] = (uint8_t)((gap > 0xFFFFu ? 0xFFFFu : gap) >> 8);
        sd[6] = (uint8_t)(exec > 255u ? 255u : exec);
#ifdef BS_PLAUS
        /* 하위 4 비트 버스오프, 상위 4 비트 타당성 검사 검출 (15 에서 멈춤) */
        sd[7] = (uint8_t)((g_dbg_busoff > 15u ? 15u : g_dbg_busoff) | ((g_bs.n_plaus > 15u ? 15u : g_bs.n_plaus) << 4));
#else
        sd[7] = (uint8_t)(g_dbg_busoff > 255u ? 255u : g_dbg_busoff);
#endif
        (void)can_send(MB_TX_SAFETY_DIAG, 0x3F4u, sd, 8u);
      }
#endif
#ifdef BMPC_PROFILE
      prof_send(APP.stats.requests);
#endif
    }
  }
}

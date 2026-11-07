/* S32K312 bmpc 펌웨어 링커 스크립트 (팀 capstone_hils 의 fw_mpc_s32k312/linker.ld 를 바탕으로).
 *
 *   flash  코드, 상수 (명령 캐시)
 *   dtcm   .data, .bss (bmpc_app = controller 작업 메모리 float 약 38 KB, 수신 링 버퍼), 스택 8 KB. 대기 없음
 *   sram   .sram_bss (입력 섹션 .bss.sram.*): APP_SRAM=1 빌드에서 bmpc_app (비교용)
 *
 * 두 RAM 은 스타트업이 ECC 초기화로 0 을 채우므로 NOLOAD 영역은 따로 지우지 않는다. */
ENTRY(Reset_Handler)

MEMORY
{
    flash (rx)  : ORIGIN = 0x00400000, LENGTH = 0x001D4000
    dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00010000
    sram  (rwx) : ORIGIN = 0x20400000, LENGTH = 0x00018000
}

_estack = ORIGIN(dtcm) + LENGTH(dtcm);
_stack_size = 0x2000;

SECTIONS
{
    .boot_header : { KEEP(*(.boot_header)) } > flash

    .vectors ORIGIN(flash) + 0x400 : { KEEP(*(.vectors)) } > flash

    .text :
    {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > flash

    __data_load = LOADADDR(.data);

    /* RAM 에서 실행하는 함수 (플래시 대기 상태 변경). 플래시에 싣고 startup_c 가 SRAM 으로 복사한다. */
    .ramfunc : AT(ALIGN(ADDR(.text) + SIZEOF(.text), 4))
    {
        . = ALIGN(4);
        __ramfunc_start = .;
        *(.ramfunc*)
        . = ALIGN(4);
        __ramfunc_end = .;
    } > sram
    __ramfunc_load = LOADADDR(.ramfunc);

    /* main.c 가 section(".bss.sram.app") 로 둔 것. 이름이 .bss. 로 시작해야 컴파일러가 NOBITS 로 만든다.
     * 아래 .bss 의 *(.bss*) 보다 먼저 와야 이쪽으로 간다. */
    .sram_bss (NOLOAD) :
    {
        . = ALIGN(32);
        __sram_bss_start = .;
        *(.bss.sram.*)
        . = ALIGN(4);
        __sram_bss_end = .;
    } > sram

    .data : AT(ALIGN(LOADADDR(.ramfunc) + SIZEOF(.ramfunc), 4))
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > dtcm

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > dtcm

    /* 스택 공간이 남는지 링크 시점에 확인 */
    ASSERT(__bss_end + _stack_size <= _estack, "DTCM 부족: .data + .bss + 스택(8 KB) > 64 KB")

    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}

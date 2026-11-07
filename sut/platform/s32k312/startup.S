/* S32K312 (Cortex-M7_0) bmpc 펌웨어 스타트업.
 * 팀 capstone_hils 의 fw_mpc_s32k312/startup.S (실보드 검증본) 를 그대로 가져왔다.
 *   - 부트 헤더, 벡터 테이블 (FlexCAN0 메일박스 0~31 인터럽트 = IRQ 110)
 *   - SRAM 96 KB (0x20400000~0x20418000) 와 DTCM 64 KB 를 ECC 때문에 먼저 0 으로 채운다.
 *     NOLOAD 영역(.bss, .sram_bss)은 여기서 이미 0 이 된다.
 *   - VTOR, FPU 켜기 뒤 startup_c (main.c) */
    .syntax unified
    .cpu cortex-m7
    .thumb

    .extern _estack
    .extern startup_c
    .extern CAN0_MB_IRQHandler

    .section .boot_header, "a"
    .word 0x5AA55AA5            /* 0x00 IVT marker */
    .word 0x00000001            /* 0x04 boot config: CM7_0 enable */
    .word 0xFFFFFFFF
    .word __vector_table        /* 0x0C CM7_0 start address */
    .word 0xFFFFFFFF
    .word 0
    .word 0xFFFFFFFF
    .word 0
    .word 0xFFFFFFFF
    .word 0
    .word 0
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF

    .section .vectors, "a"
    .balign 1024
    .global __vector_table
__vector_table:
    .word _estack
    .word Reset_Handler
    .word Default_Handler       /* NMI */
    .word Fault_Handler         /* HardFault */
    .word Fault_Handler         /* MemManage */
    .word Fault_Handler         /* BusFault */
    .word Fault_Handler         /* UsageFault */
    .word 0, 0, 0, 0
    .word Default_Handler       /* SVC */
    .word Default_Handler       /* DebugMon */
    .word 0
    .word Default_Handler       /* PendSV */
    .word SysTick_Handler       /* SysTick (안전 제어기 통합 빌드만 정의, 아니면 Default_Handler) */
    /* 외부 인터럽트 0..109 */
    .rept 110
    .word Default_Handler
    .endr
    .word CAN0_MB_IRQHandler    /* IRQ 110: FlexCAN0 MB 0-31 */
    .rept 17
    .word Default_Handler
    .endr

    .text
    .thumb_func
    .global Reset_Handler
Reset_Handler:
    cpsid i
    /* SRAM 은 ECC 때문에 64 비트 단위로 먼저 써 줘야 한다. 스택을 쓰기 전에 96 KB 전체를 0 으로 채운다. */
    ldr   r0, =0x20400000
    ldr   r1, =0x20418000
    movs  r2, #0
    movs  r3, #0
    movs  r4, #0
    movs  r5, #0
    movs  r6, #0
    movs  r7, #0
    movs  r8, #0
    movs  r9, #0
1:  stmia r0!, {r2-r9}
    cmp   r0, r1
    blo   1b

    /* DTCM 64 KB (0x20000000~0x20010000) 도 ECC 라 먼저 0 으로 쓴다. 데이터와 스택은 DTCM 에 둔다(대기 없음) */
    ldr   r0, =0x20000000
    ldr   r1, =0x20010000
3:  stmia r0!, {r2-r9}
    cmp   r0, r1
    blo   3b

    ldr   sp, =_estack

    /* VTOR = 이 벡터 테이블 */
    ldr   r0, =0xE000ED08
    ldr   r1, =__vector_table
    str   r1, [r0]

    /* FPU(CP10/CP11) */
    ldr   r0, =0xE000ED88
    ldr   r1, [r0]
    orr   r1, r1, #(0xF << 20)
    str   r1, [r0]
    dsb
    isb

    bl    startup_c
2:  b     2b

    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .thumb_func
    .global Default_Handler
Default_Handler:
    b     Default_Handler

    /* 폴트: 디버거로 원인 확인용 (g_fault_* 를 읽는다) */
    .thumb_func
    .global Fault_Handler
Fault_Handler:
    b     Fault_Handler

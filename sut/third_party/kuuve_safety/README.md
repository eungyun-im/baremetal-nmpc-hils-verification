# kuuve_safety

팀 `capstone_hils`의 안전 제어기(DUT) `safety/safety_ctrl.{c,h}`를 **수정 없이** 가져온 사본입니다(2026-10-28 스냅샷 `capstone_hils_20261028`).
요구사항(TSR-01~13), 단위 시험, MC/DC, 변조 시험은 원본 저장소에 있습니다. 바꿀 일이 생기면 원본에서 고치고 다시 복사합니다.

bmpc 펌웨어에서는 `integ/bmpc_safety`가 MPC 출력을 이 안전 제어기의 명령(CTRL_CMD) 형식으로 바꿔 넣고,
10 ms마다 `sc_step`으로 검사한 값(SAFE_OUT)을 차량 명령으로 내보냅니다.

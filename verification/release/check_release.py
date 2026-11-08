#!/usr/bin/env python3
"""Release gate for the S32K312 firmware ELF.

A release build must not contain test-bench features. Checks (all must pass):
  R1  feature word g_fw_features: SAFETY, PLAUS and SWT on; FAULT_HOOK, BENCH_RESET and PROFILE off
  R2  the CAN receive table k_rx_id has no mailbox for BENCH_RESET (0x7F0)
  R3  no code compares a value with 0x7F0 (the BENCH_RESET handler is compiled out)
  R4  build id is set and the tree was clean (not "unknown", not "-dirty")
  R5  no software double-precision routines (__aeabi_d*), which the FPU cannot run in hardware

usage: check_release.py fw_bmpc.elf [--expect-fail] [--tools <dir with arm-none-eabi-*>]
--expect-fail inverts the exit code, used to show that a bench build is rejected.
"""
import argparse, os, re, shutil, struct, subprocess, sys

FEATURES = {0: "SAFETY", 1: "PLAUS", 2: "SWT", 3: "PLL", 4: "DCACHE", 8: "FAULT_HOOK", 9: "BENCH_RESET", 10: "PROFILE"}
REQUIRED = {"SAFETY", "PLAUS", "SWT"}
FORBIDDEN = {"FAULT_HOOK", "BENCH_RESET", "PROFILE"}
BENCH_RESET_ID = 0x7F0


def tool(tools, name):
    p = os.path.join(tools, "arm-none-eabi-" + name) if tools else shutil.which("arm-none-eabi-" + name)
    if not p or not os.path.exists(p):
        sys.exit(f"arm-none-eabi-{name} not found (use --tools)")
    return p


def read_at(elf, addr, size):
    """Read bytes at a virtual address from an ELF32 little-endian file (section table lookup)."""
    data = open(elf, "rb").read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise ValueError("expected ELF32 little-endian")
    shoff, = struct.unpack_from("<I", data, 0x20)
    shentsize, shnum = struct.unpack_from("<HH", data, 0x2E)
    for i in range(shnum):
        _, sh_type, _, sh_addr, sh_offset, sh_size = struct.unpack_from("<IIIIII", data, shoff + i * shentsize)
        if sh_type == 1 and sh_addr <= addr and addr + size <= sh_addr + sh_size:  # PROGBITS
            off = sh_offset + addr - sh_addr
            return data[off:off + size]
    raise ValueError(f"address {addr:#x} not in a loaded section")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--tools", default=None)
    ap.add_argument("--expect-fail", action="store_true")
    a = ap.parse_args()

    syms = {}
    for line in subprocess.run([tool(a.tools, "nm"), "-S", a.elf], capture_output=True, text=True, check=True).stdout.splitlines():
        p = line.split()
        if len(p) == 4:
            syms[p[3]] = (int(p[0], 16), int(p[1], 16))
        elif len(p) == 3:
            syms[p[2]] = (int(p[0], 16), 0)
    results = []

    def rule(rid, ok, msg):
        results.append((rid, ok, msg))

    if "g_fw_features" in syms:
        word, = struct.unpack("<I", read_at(a.elf, syms["g_fw_features"][0], 4))
        on = {n for b, n in FEATURES.items() if word >> b & 1}
        bad = (REQUIRED - on) | (FORBIDDEN & on)
        rule("R1", not bad, f"features {word:#06x} = {sorted(on)}" + (f"; violates {sorted(bad)}" if bad else ""))
    else:
        rule("R1", False, "g_fw_features not found (old firmware without build identity?)")

    if "k_rx_id" in syms and syms["k_rx_id"][1]:
        addr, size = syms["k_rx_id"]
        ids = struct.unpack(f"<{size // 4}I", read_at(a.elf, addr, size))
        rule("R2", BENCH_RESET_ID not in ids, "receive IDs " + " ".join(f"{i:#05x}" for i in ids))
    else:
        rule("R2", False, "k_rx_id not found")

    dis = subprocess.run([tool(a.tools, "objdump"), "-d", a.elf], capture_output=True, text=True, check=True).stdout
    cmps = [l.strip() for l in dis.splitlines() if re.search(r"\bcmp(\.w)?\s+r\d+, #2032\b", l)]
    rule("R3", not cmps, f"{len(cmps)} comparisons with 0x7F0" + (": " + cmps[0] if cmps else ""))

    if "g_fw_build_id" in syms:
        addr, size = syms["g_fw_build_id"]
        bid = read_at(a.elf, addr, size).split(b"\0")[0].decode(errors="replace")
        rule("R4", bid not in ("", "unknown") and not bid.endswith("-dirty"), f"build id '{bid}'")
    else:
        rule("R4", False, "g_fw_build_id not found")

    soft = sorted(s for s in syms if re.fullmatch(r"__aeabi_(d[a-z]+|f2d|d2f|[iul]+2d|d2[iul]+z)", s))
    rule("R5", not soft, "software double routines: " + (", ".join(soft) if soft else "none"))

    for rid, ok, msg in results:
        print(f"{'PASS' if ok else 'FAIL'} {rid} {msg}")
    passed = all(ok for _, ok, _ in results)
    print(("release gate PASSED" if passed else "release gate FAILED") + (" (expected to fail)" if a.expect_fail else ""))
    sys.exit(0 if passed != a.expect_fail else 1)


if __name__ == "__main__":
    main()

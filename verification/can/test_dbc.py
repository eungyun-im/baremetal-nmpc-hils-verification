#!/usr/bin/env python3
"""Check bmpc.dbc against the firmware's C encoder and against a real recording.

1. Builds dbc_vectors.c with the real encoder (double and float builds), decodes every
   frame with cantools, and compares each signal with what the C decoder returns.
2. Checks the CRC-16/CCITT-FALSE described in the DBC comments.
3. Decodes sample_frames.bin (a real SILS recording) and checks every request/response
   group decodes and its CRC matches.
Exit code 0 = pass.
"""
import math, os, struct, subprocess, sys, tempfile
import cantools

HERE = os.path.dirname(os.path.abspath(__file__))
SUT = os.path.join(HERE, "..", "..", "sut")
DB = cantools.database.load_file(os.path.join(HERE, "bmpc.dbc"), strict=True)
fails = []


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def close(a, b):
    return abs(a - b) <= 1e-9 + 1e-6 * max(abs(a), abs(b))


def check(tag, got, want):
    for k, v in want.items():
        if k not in got:
            fails.append(f"{tag}: signal {k} missing in DBC decode")
        elif not close(float(got[k]), float(v)):
            fails.append(f"{tag}: {k} dbc={got[k]} c={v}")


def build(flags):
    exe = tempfile.mktemp(prefix="dbcvec")
    src = [os.path.join(HERE, "dbc_vectors.c"), os.path.join(SUT, "core/src/bmpc_protocol.c")]
    subprocess.run(["gcc", "-std=c99", "-O1", *flags, "-I", os.path.join(SUT, "core/include"), *src, "-lm", "-o", exe], check=True)
    out = subprocess.run([exe, "300"], check=True, capture_output=True, text=True).stdout
    os.unlink(exe)
    return out


def run_vectors(label, out):
    frames, n_groups = [], 0
    for line in out.splitlines():
        p = line.split()
        if p[0] == "F":
            frames.append((int(p[1], 16), bytes(int(x, 16) for x in p[2:])))
            continue
        kind, vals = p[1], dict(kv.split("=") for kv in p[2:])
        decoded, raw = {}, {}
        for fid, data in frames:
            msg = DB.get_message_by_frame_id(fid)
            decoded.update(msg.decode(data, decode_choices=False))
            raw.update(msg.decode(data, decode_choices=False, scaling=False))
        tag = f"{label} {kind} #{n_groups}"
        if kind == "REQ":
            flags = sum(int(decoded[n]) << i for i, n in enumerate(["FlagReset", "FlagHighway", "FlagAvoidance", "FlagKick", "FlagStop"]))
            if flags != int(vals.pop("Flags")):
                fails.append(f"{tag}: flags")
            for opt in ("PostCap", "Snap"):
                if vals.pop(opt + "Raw") == "-1" and raw[opt] != 0xFFFF:
                    fails.append(f"{tag}: {opt} 'none' should be raw 0xFFFF")
            check(tag, decoded, vals)
            body = b"".join(d for _, d in frames)
            if crc16(body[:-2]) != raw["ReqCrc16"]:
                fails.append(f"{tag}: request CRC")
        elif kind == "REQ2":
            for opt in ("PostCap", "Snap"):
                if float(vals[opt]) >= 0:
                    check(tag, decoded, {opt: vals[opt]})
            frames = []
            continue
        elif kind == "RESP":
            check(tag, decoded, vals)
            body = b"".join(d for _, d in frames)
            if crc16(body[:-2]) != raw["RespCrc16"]:
                fails.append(f"{tag}: response CRC")
        else:
            check(tag, decoded, vals)
        if kind != "REQ":
            frames = []
        n_groups += 1
    return n_groups


def run_recording(path):
    """frames_<epoch>.bin: records of int64 t_us, u8 dir (0 rx at MCU, 1 tx), u32 id, u8 dlc, 8 data bytes."""
    rec = struct.Struct("<qBIB8s")
    data = open(path, "rb").read()
    req, resp, n_req, n_resp, unknown = {}, {}, 0, 0, set()
    for off in range(0, len(data) - rec.size + 1, rec.size):
        t, d, fid, dlc, payload = rec.unpack_from(data, off)
        try:
            DB.get_message_by_frame_id(fid).decode(payload)
        except KeyError:
            unknown.add(fid)
            continue
        if 0x100 <= fid <= 0x105:
            req[fid] = payload
            if fid == 0x105 and len(req) == 6:
                body = b"".join(req[i] for i in range(0x100, 0x106))
                if crc16(body[:-2]) != struct.unpack_from("<H", body, 46)[0]:
                    fails.append(f"recording: request CRC at t={t}")
                n_req += 1
                req = {}
        elif 0x1E0 <= fid <= 0x1E3:
            resp[fid] = payload
            if fid == 0x1E3 and len(resp) == 4:
                body = b"".join(resp[i] for i in range(0x1E0, 0x1E4))
                if crc16(body[:-2]) != struct.unpack_from("<H", body, 30)[0]:
                    fails.append(f"recording: response CRC at t={t}")
                n_resp += 1
                resp = {}
    if unknown:
        fails.append("recording: frame IDs not in bmpc.dbc: " + ", ".join(hex(i) for i in sorted(unknown)))
    if n_req == 0 or n_resp == 0:
        fails.append("recording: no complete request/response groups")
    return n_req, n_resp


if __name__ == "__main__":
    for label, flags in (("double", []), ("float", ["-DBMPC_USE_FLOAT"])):
        n = run_vectors(label, build(flags))
        print(f"{label}: {n} encoded messages decoded with the DBC")
    sample = os.path.join(HERE, "sample_frames.bin")
    if os.path.exists(sample):
        print("recording: %d requests, %d responses decoded, CRC checked" % run_recording(sample))
        sys.path.insert(0, HERE)
        import can
        from frames_to_log import convert, read_frames
        want = [(m.arbitration_id, bytes(m.data), m.is_rx) for m in read_frames(sample)]
        for ext in (".asc", ".blf"):
            dst = tempfile.mktemp(suffix=ext)
            convert(sample, dst)
            got = [(m.arbitration_id, bytes(m.data), m.is_rx) for m in can.LogReader(dst)]
            os.unlink(dst)
            if got != want:
                fails.append(f"{ext}: round trip differs ({len(got)} vs {len(want)} frames)")
            else:
                print(f"{ext}: {len(got)} frames written and read back unchanged")
    for f in fails[:20]:
        print("FAIL", f)
    print("PASS" if not fails else f"{len(fails)} failures")
    sys.exit(1 if fails else 0)

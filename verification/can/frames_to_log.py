#!/usr/bin/env python3
"""Convert a bmpc frame recording (frames_<epoch>.bin from sim_mcu --record) to Vector ASC or BLF,
so it opens in CANoe/CANalyzer, SavvyCAN or python-can tools together with bmpc.dbc.

Record format (little-endian, 22 bytes): int64 t_us | u8 dir (0 = received by the MCU, 1 = sent by the MCU) |
u32 id | u8 dlc | 8 data bytes.

usage: frames_to_log.py frames_xxx.bin out.asc|out.blf [--channel 0] [--start-epoch S]
"""
import argparse, re, struct
import can

REC = struct.Struct("<qBIB8s")


def read_frames(path, channel=0, start_epoch=None):
    """channel is 0-based (python-can writes it as CAN 1 in ASC). t_us is a monotonic clock, so timestamps are
    shifted to start at start_epoch (default: the <epoch> in the file name, else 0)."""
    data = open(path, "rb").read()
    if start_epoch is None:
        m = re.search(r"frames_(\d+)\.bin$", path)
        start_epoch = int(m.group(1)) if m else 0
    t_first = REC.unpack_from(data, 0)[0] if data else 0
    if len(data) % REC.size:
        raise ValueError(f"{path}: size {len(data)} is not a multiple of {REC.size}")
    for off in range(0, len(data), REC.size):
        t_us, direction, fid, dlc, payload = REC.unpack_from(data, off)
        yield can.Message(timestamp=start_epoch + (t_us - t_first) / 1e6, arbitration_id=fid, is_extended_id=fid > 0x7FF,
                          is_rx=(direction == 0), dlc=dlc, data=payload[:dlc], channel=channel)


def convert(src, dst, channel=0, start_epoch=None):
    n = 0
    with can.Logger(dst) as log:
        for msg in read_frames(src, channel, start_epoch):
            log.on_message_received(msg)
            n += 1
    return n


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--channel", type=int, default=0, help="0-based, shown as CAN 1 in ASC")
    ap.add_argument("--start-epoch", type=float, default=None)
    a = ap.parse_args()
    print(f"{convert(a.src, a.dst, a.channel, a.start_epoch)} frames -> {a.dst}")

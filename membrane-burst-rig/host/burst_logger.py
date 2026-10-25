#!/usr/bin/env python3
"""burst_logger.py - laptop side of the membrane burst rig.

Reads the RP2040 burst_capture stream and writes one folder per run:

    runs/<YYYYmmdd_HHMMSS>_<label>/
        run.json            label, notes, firmware header, start time
        slow.csv            host_time, ms, ch0, ch1, ...   (~1 kHz 1 ms means)
        capture_001.npz     counts[frames, n_ch] uint16 (12-bit, error bit stripped)
        capture_001.json    fs_ch, trigger frame, flags, host time, label, notes

Usage:
    pip install pyserial numpy
    python burst_logger.py COM5 --label mylar_0p5mil --notes "base volume"
    python burst_logger.py /dev/ttyACM0 --label latex_single

While running, type a command and press Enter; it is sent to the board:
    t      force a capture (pin the membrane within ~200 ms)
    d20    set the drop trigger to 20 counts
    ?      re-print the header
    q      quit
"""
import argparse
import datetime as dt
import json
import os
import queue
import sys
import threading
import time

import numpy as np


class StreamParser:
    """Splits the device byte stream into header / slow / info lines and binary captures."""

    def __init__(self):
        self.buf = bytearray()
        self.cap_hdr = None          # dict while waiting for capture payload
        self.need = 0

    def feed(self, data):
        """Feed raw bytes; yields ('header', dict) | ('slow', list) | ('info', str) | ('capture', dict, ndarray)."""
        self.buf += data
        while True:
            if self.cap_hdr is not None:
                if len(self.buf) < self.need + 3:
                    return
                payload = bytes(self.buf[: self.need])
                tail = bytes(self.buf[self.need : self.need + 3])
                del self.buf[: self.need + 3]
                hdr, self.cap_hdr = self.cap_hdr, None
                raw = np.frombuffer(payload, dtype="<u2").reshape(hdr["frames"], hdr["n_ch"])
                hdr["err_samples"] = int(np.count_nonzero(raw & 0x8000))
                hdr["framing_ok"] = tail == b"\nE\n"
                yield ("capture", hdr, (raw & 0x0FFF).astype(np.uint16))
                continue
            nl = self.buf.find(b"\n")
            if nl < 0:
                return
            line = self.buf[:nl].decode("ascii", "replace").strip()
            del self.buf[: nl + 1]
            if not line:
                continue
            if line.startswith("C,"):
                f = line.split(",")
                hdr = dict(number=int(f[1]), frames=int(f[2]), n_ch=int(f[3]),
                           trig_frame=int(f[4]), fs_ch=float(f[5]), forced=bool(int(f[6])))
                self.cap_hdr = hdr
                self.need = hdr["frames"] * hdr["n_ch"] * 2
            elif line.startswith("S,"):
                try:
                    yield ("slow", [float(x) for x in line.split(",")[1:]])
                except ValueError:
                    pass
            elif line.startswith("{"):
                try:
                    yield ("header", json.loads(line))
                except json.JSONDecodeError:
                    yield ("info", line)
            else:
                yield ("info", line)


def stdin_reader(q):
    for line in sys.stdin:
        q.put(line.strip())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--label", default="run", help="short run label, used in the folder name")
    ap.add_argument("--notes", default="", help="free-text notes stored with every capture")
    ap.add_argument("--outdir", default="runs", help="parent folder for run folders")
    ap.add_argument("--no-slow", action="store_true", help="do not write slow.csv")
    args = ap.parse_args()

    import serial  # pyserial

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in args.label)
    run_dir = os.path.join(args.outdir, f"{stamp}_{safe}")
    os.makedirs(run_dir, exist_ok=True)

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    ser.reset_input_buffer()
    ser.write(b"?\n")

    run_meta = dict(label=args.label, notes=args.notes, started=dt.datetime.now().isoformat(),
                    port=args.port, header=None)
    slow = None if args.no_slow else open(os.path.join(run_dir, "slow.csv"), "w", buffering=1)
    parser = StreamParser()
    cmds = queue.Queue()
    threading.Thread(target=stdin_reader, args=(cmds,), daemon=True).start()
    n_caps = 0

    def save_meta():
        with open(os.path.join(run_dir, "run.json"), "w") as f:
            json.dump(run_meta, f, indent=2)

    save_meta()
    print(f"logging to {run_dir}   (commands: t, d<N>, ?, q)")
    try:
        while True:
            while not cmds.empty():
                c = cmds.get()
                if c == "q":
                    raise KeyboardInterrupt
                if c:
                    ser.write((c + "\n").encode())
            data = ser.read(65536)
            if not data:
                continue
            for item in parser.feed(data):
                kind = item[0]
                if kind == "header":
                    run_meta["header"] = item[1]
                    save_meta()
                    if slow is not None and slow.tell() == 0:
                        n = item[1]["n_ch"]
                        slow.write("host_time,ms," + ",".join(f"ch{i}" for i in range(n)) + "\n")
                    print("header:", item[1])
                elif kind == "slow" and slow is not None:
                    slow.write(f"{time.time():.4f}," + ",".join(f"{v:g}" for v in item[1]) + "\n")
                elif kind == "info":
                    print("board:", item[1])
                elif kind == "capture":
                    hdr, counts = item[1], item[2]
                    n_caps += 1
                    base = os.path.join(run_dir, f"capture_{n_caps:03d}")
                    np.savez_compressed(base + ".npz", counts=counts)
                    hdr.update(host_time=dt.datetime.now().isoformat(), label=args.label, notes=args.notes,
                               rail_ch=(run_meta["header"] or {}).get("rail_ch", -1),
                               firmware=(run_meta["header"] or {}).get("fw"))
                    with open(base + ".json", "w") as f:
                        json.dump(hdr, f, indent=2)
                    pre = counts[: hdr["trig_frame"]].mean(axis=0)
                    end = counts[-hdr["frames"] // 10 :].mean(axis=0)
                    flag = "" if hdr["framing_ok"] else "  FRAMING ERROR"
                    print(f"capture {n_caps:3d}{' (forced)' if hdr['forced'] else ''}: "
                          f"pre {np.round(pre).astype(int).tolist()}  end {np.round(end).astype(int).tolist()}"
                          f"  adc_err={hdr['err_samples']}{flag}")
    except KeyboardInterrupt:
        pass
    finally:
        run_meta["ended"] = dt.datetime.now().isoformat()
        run_meta["captures"] = n_caps
        save_meta()
        if slow is not None:
            slow.close()
        ser.close()
        print(f"\n{n_caps} capture(s) in {run_dir}")


if __name__ == "__main__":
    main()

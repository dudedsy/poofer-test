#!/usr/bin/env python3
"""burst_logger.py - laptop side of the membrane burst rig.

Two jobs:

Live logging (default). Puts the board in Live mode and records what it streams:

    runs/<YYYYmmdd_HHMMSS>_<label>/
        run.json            label, notes, firmware header, start time
        slow.csv            host_time, ms, ch0, ch1, ...   (~1 kHz 1 ms means)
        capture_001.npz     counts[frames, n_ch] uint16 (12-bit, error bit stripped)
        capture_001.json    fs_ch, trigger frame, flags, host time, label, notes

    python burst_logger.py COM5 --label mylar_0p5mil --notes "base volume"
    python burst_logger.py /dev/ttyACM0 --label latex_single --channels 2

    While running, type a command and press Enter; it is sent to the board:
        t      force a capture (the burst has to happen within ~80 ms)
        d20    set the drop trigger to 20 counts
        ?      re-print the header
        q      quit

Dump (--dump). Copies captures stored on the board in Collect mode. The board must
be in the menu or Dump mode (not Collect). One folder per tag set on the board:

    runs/<YYYYmmdd_HHMMSS>_<tag label>/capture_<id>.npz / .json

    python burst_logger.py COM5 --dump --tags 1=mylar_base,2=mylar_capped
    python burst_logger.py COM5 --dump --erase      # delete each one once verified

    Captures already present under --outdir (same board id and CRC) are not copied
    again, so dumping twice is harmless. --erase only deletes a capture after its
    copy is written and its CRC, framing and length check out.
"""
import argparse
import datetime as dt
import glob
import json
import os
import queue
import sys
import threading
import time

import numpy as np


class StreamParser:
    """Splits the device byte stream into header / slow / info / json lines and binary captures."""

    def __init__(self):
        self.buf = bytearray()
        self.cap_hdr = None          # dict while waiting for capture payload
        self.need = 0

    def feed(self, data):
        """Feed raw bytes; yields ('header', dict) | ('slow', list) | ('info', str) |
        ('json', dict) | ('capture', dict, ndarray)."""
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
            elif line.startswith("J,"):
                try:
                    yield ("json", json.loads(line[2:]))
                except json.JSONDecodeError:
                    yield ("info", line)
            elif line.startswith("{"):
                try:
                    yield ("header", json.loads(line))
                except json.JSONDecodeError:
                    yield ("info", line)
            else:
                yield ("info", line)


def safe_name(s):
    return "".join(c if c.isalnum() or c in "-_." else "_" for c in s)


def stdin_reader(q):
    for line in sys.stdin:
        q.put(line.strip())


# ---------------------------------------------------------------- live logging
def run_live(ser, args):
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.outdir, f"{stamp}_{safe_name(args.label)}")
    os.makedirs(run_dir, exist_ok=True)

    ser.write(b"ML\n")                       # Live mode (ignored by older firmware)
    if args.channels:
        ser.write(f"N{args.channels}\n".encode())
    ser.write(b"?\n")

    run_meta = dict(label=args.label, notes=args.notes, started=dt.datetime.now().isoformat(),
                    port=args.port, header=None)
    slow = None if args.no_slow else open(os.path.join(run_dir, "slow.csv"), "w", buffering=1)
    parser = StreamParser()
    cmds = queue.Queue()
    threading.Thread(target=stdin_reader, args=(cmds,), daemon=True).start()
    n_caps = 0
    live_meta = {}

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
                elif kind == "json" and item[1].get("t") == "live_meta":
                    live_meta = {k: v for k, v in item[1].items() if k not in ("t", "num")}
                elif kind in ("info", "json"):
                    print("board:", item[1])
                elif kind == "capture":
                    hdr, counts = item[1], item[2]
                    n_caps += 1
                    base = os.path.join(run_dir, f"capture_{n_caps:03d}")
                    np.savez_compressed(base + ".npz", counts=counts)
                    hdr.update(host_time=dt.datetime.now().isoformat(), label=args.label, notes=args.notes,
                               rail_ch=(run_meta["header"] or {}).get("rail_ch", -1),
                               firmware=(run_meta["header"] or {}).get("fw"), source="live", **live_meta)
                    live_meta = {}
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
        print(f"\n{n_caps} capture(s) in {run_dir}")


# ---------------------------------------------------------------- dump
class Device:
    """Request/response helper over the board's serial protocol."""

    def __init__(self, ser, verbose=True):
        self.ser, self.parser, self.verbose = ser, StreamParser(), verbose
        self.pending = []

    def _items(self, timeout):
        deadline = time.time() + timeout
        while True:
            while self.pending:
                yield self.pending.pop(0)
            if time.time() > deadline:
                return
            data = self.ser.read(65536)
            if data:
                self.pending.extend(self.parser.feed(data))
                deadline = max(deadline, time.time() + 1.0)   # data still flowing

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def wait_json(self, types, timeout=5.0):
        for it in self._items(timeout):
            if it[0] == "json" and it[1].get("t") in types:
                return it[1]
            if it[0] == "json" and it[1].get("t") == "error":
                raise RuntimeError(it[1].get("msg", "device error"))
            if it[0] == "info" and self.verbose:
                print("board:", it[1])
        raise TimeoutError(f"no reply ({'/'.join(types)})")

    def list(self):
        self.send("L")
        entries = []
        for it in self._items(10.0):
            if it[0] != "json":
                if it[0] == "info" and self.verbose:
                    print("board:", it[1])
                continue
            j = it[1]
            if j.get("t") == "entry":
                entries.append(j)
            elif j.get("t") == "list_end":
                return entries, j
            elif j.get("t") == "error":
                raise RuntimeError(j.get("msg", "device error"))
        raise TimeoutError("no list from the board (old firmware, or not responding?)")

    def read(self, cid):
        self.send(f"R{cid}")
        meta = self.wait_json(("meta",))
        for it in self._items(20.0):
            if it[0] == "capture":
                return meta, it[1], it[2]
            if it[0] == "info" and self.verbose:
                print("board:", it[1])
        raise TimeoutError(f"capture {cid}: no data")

    def delete(self, cid):
        self.send(f"X{cid}")
        j = self.wait_json(("deleted",))
        return bool(j.get("ok")) and j.get("id") == cid


def parse_tags(s):
    out = {}
    for part in filter(None, (s or "").split(",")):
        k, _, v = part.partition("=")
        out[int(k)] = v.strip()
    return out


def already_dumped(outdir):
    """{(device id, data_crc): path} for captures copied from flash earlier."""
    seen = {}
    for fn in glob.glob(os.path.join(outdir, "*", "capture_*.json")):
        try:
            j = json.load(open(fn))
        except (OSError, json.JSONDecodeError):
            continue
        if j.get("source") == "flash" and os.path.exists(fn[:-5] + ".npz"):
            seen[(j.get("id"), j.get("data_crc"))] = fn
    return seen


def run_dump(ser, args):
    dev = Device(ser)
    ser.reset_input_buffer()
    entries, end = dev.list()
    print(f"board has {end['count']} stored capture(s), room for {end['free']} more")
    if not entries:
        return 0
    tags = parse_tags(args.tags)
    seen = already_dumped(args.outdir)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    folders = {}
    copied = skipped = erased = failed = 0

    for e in entries:
        cid = e["id"]
        prev = seen.get((cid, e.get("data_crc")))
        ok = False
        if prev:
            print(f"#{cid:04d}: already copied ({os.path.relpath(prev, args.outdir)})")
            skipped += 1
            ok = True
        else:
            try:
                meta, hdr, counts = dev.read(cid)
            except (TimeoutError, RuntimeError) as ex:
                print(f"#{cid:04d}: read failed: {ex}")
                failed += 1
                continue
            problems = []
            if not hdr["framing_ok"]:
                problems.append("framing error")
            if not meta.get("crc_ok"):
                problems.append("CRC mismatch in flash")
            if counts.shape != (meta["frames"], meta["n_ch"]):
                problems.append(f"shape {counts.shape} != ({meta['frames']}, {meta['n_ch']})")
            tag = meta.get("tag", 0)
            label = tags.get(tag, f"{args.label}_tag{tag}" if args.label != "run" else f"tag{tag}")
            if tag not in folders:
                d = os.path.join(args.outdir, f"{stamp}_{safe_name(label)}")
                os.makedirs(d, exist_ok=True)
                with open(os.path.join(d, "run.json"), "w") as f:
                    json.dump(dict(label=label, tag=tag, notes=args.notes, source="flash dump",
                                   dumped=dt.datetime.now().isoformat(), port=args.port), f, indent=2)
                folders[tag] = d
            base = os.path.join(folders[tag], f"capture_{cid:04d}")
            np.savez_compressed(base + ".npz", counts=counts)
            out = dict(meta)
            out.pop("t", None)
            out.update(number=cid, frames=meta["frames"], n_ch=meta["n_ch"], trig_frame=meta["trig_frame"],
                       fs_ch=float(meta["fs_ch"]), forced=bool(meta["forced"]), framing_ok=hdr["framing_ok"],
                       rail_ch=-1, label=label, notes=args.notes, source="flash",
                       host_time=dt.datetime.now().isoformat(), problems=problems)
            with open(base + ".json", "w") as f:
                json.dump(out, f, indent=2)
            p0 = ", ".join(f"{p:.1f}" for p in meta.get("p0_psi", []))
            print(f"#{cid:04d}: tag {tag} -> {os.path.relpath(base, args.outdir)}  p0 [{p0}] psi"
                  f"{'  forced' if meta['forced'] else ''}"
                  f"{'  adc_err=' + str(meta['err_samples']) if meta.get('err_samples') else ''}"
                  f"{'  PROBLEM: ' + '; '.join(problems) if problems else ''}")
            copied += 1
            ok = not problems
            if problems:
                failed += 1
        if ok and args.erase:
            if dev.delete(cid):
                erased += 1
            else:
                print(f"#{cid:04d}: delete failed")

    print(f"\ncopied {copied}, already had {skipped}, problems {failed}"
          + (f", erased {erased} from the board" if args.erase else ""))
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--label", default="run", help="short run label, used in the folder name")
    ap.add_argument("--notes", default="", help="free-text notes stored with every capture")
    ap.add_argument("--outdir", default="runs", help="parent folder for run folders")
    ap.add_argument("--no-slow", action="store_true", help="do not write slow.csv")
    ap.add_argument("--channels", type=int, choices=(1, 2, 3), help="set the board's channel count (live)")
    ap.add_argument("--dump", action="store_true", help="copy captures stored on the board")
    ap.add_argument("--tags", default="", help="dump: tag labels, e.g. 1=mylar_base,2=mylar_capped")
    ap.add_argument("--erase", action="store_true", help="dump: delete each capture from the board once verified")
    args = ap.parse_args()

    import serial  # pyserial

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    ser.reset_input_buffer()
    try:
        if args.dump:
            sys.exit(run_dump(ser, args))
        run_live(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()

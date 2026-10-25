import sys, numpy as np
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'host'))
from burst_logger import StreamParser
rng = np.random.default_rng(0)
frames, nch = 25000, 3
data = rng.integers(0, 4096, (frames, nch)).astype('<u2')
data[10, 1] |= 0x8000                      # one ADC error flag
stream = (b'{"fw":"burst_capture 2.0","n_ch":3,"fs_ch":100000,"rail_ch":-1}\r\n'
          b'S,1,410.0,411.5,409.9\r\n#trig_drop=20\r\n'
          + f"C,1,{frames},{nch},5000,100000,0\r\n".encode() + data.tobytes() + b"\nE\n"
          + b'S,2,410.1,411.4,409.8\r\n')
p = StreamParser(); out = []
i = 0
while i < len(stream):                      # random chunking, like serial reads
    n = int(rng.integers(1, 9000)); out += list(p.feed(stream[i:i+n])); i += n
kinds = [o[0] for o in out]
print(kinds)
cap = [o for o in out if o[0] == "capture"][0]
assert cap[1]["framing_ok"] and cap[1]["err_samples"] == 1 and cap[1]["trig_frame"] == 5000
assert np.array_equal(cap[2], data & 0x0FFF)
assert kinds == ["header", "slow", "info", "capture", "slow"]
print("parser OK")

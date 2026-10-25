"""simport.py - run tools/sim/burst_sim as a child process and present it as a
pyserial-like port, so host/burst_logger.py code can talk to the simulated board.

    from simport import SimPort
    port = SimPort()                 # starts ./burst_sim
    port.sim("!p 30 2000")           # simulator command
    port.write(b"L\\n")              # board serial
    data = port.read(65536)
"""
import os
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))


class SimPort:
    def __init__(self, args=(), exe=None, stderr_path=None):
        exe = exe or os.path.join(HERE, "burst_sim")
        self.err = open(stderr_path, "wb") if stderr_path else subprocess.DEVNULL
        self.p = subprocess.Popen([exe, *args], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=self.err, bufsize=0)
        self.buf = bytearray()
        self.lock = threading.Lock()
        threading.Thread(target=self._pump, daemon=True).start()
        self.timeout = 0.05

    def _pump(self):
        while True:
            b = self.p.stdout.read1(65536) if hasattr(self.p.stdout, "read1") else os.read(self.p.stdout.fileno(), 65536)
            if not b:
                return
            with self.lock:
                self.buf += b

    # --- pyserial subset ---
    def write(self, data):
        self.p.stdin.write(data)
        self.p.stdin.flush()
        return len(data)

    def read(self, n=1):
        deadline = time.time() + self.timeout
        while True:
            with self.lock:
                if self.buf:
                    out = bytes(self.buf[:n])
                    del self.buf[:n]
                    return out
            if time.time() > deadline:
                return b""
            time.sleep(0.002)

    def reset_input_buffer(self):
        with self.lock:
            self.buf.clear()

    def close(self):
        try:
            self.sim("!quit")
            self.p.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        self.p.wait(timeout=30)

    # --- simulator control ---
    def sim(self, cmd):
        self.write((cmd + "\n").encode())

    def run(self, cmd, settle=0.3):
        """Send a simulator command and give the simulator a moment to finish it."""
        self.sim(cmd)
        self.sim("!mark sync")
        time.sleep(settle)

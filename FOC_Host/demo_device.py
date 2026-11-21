"""显式 --demo 模式的虚拟设备；不会打开串口或控制实际电机。"""
import math
import time


class DemoSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.host = False
        self.last = time.monotonic()
        self.next_frame = self.last
        self.calibration = None
        self.enabled = [False, False]
        self.calibrated = [False, False]
        self.cogging_valid = [False, False]
        self.tuning_saved = [False, False]
        self.tuning = [None, None]
        self.dirty = [False, False]
        self.rpm = [0.0, 0.0]
        self.target = [0.0, 0.0]
        self.position = [47.2, 210.4]
        base = dict(phase_resistance_ohm="0.12", ld_h="0.00005", lq_h="0.00005",
                    flux_linkage_wb="0.0021", pole_pairs="7", current_limit_a="8",
                    speed_kp="0.2", speed_ki="1", cogging_gain="1")
        self.params = [base.copy(), base.copy()]
        self.persisted = [base.copy(), base.copy()]

    def put(self, line):
        self.buffer.extend((line + "\r\n").encode("ascii"))

    def _pump(self):
        now = time.monotonic()
        dt = now-self.last
        self.last = now
        for axis in (0, 1):
            goal = self.target[axis] if self.enabled[axis] else 0
            self.rpm[axis] += (goal-self.rpm[axis]) * (1-math.exp(-dt/.3))
            self.position[axis] = (self.position[axis] + self.rpm[axis]*6*dt) % 360
        if self.calibration is not None:
            axis, deadline, phase = self.calibration
            if now >= deadline:
                if phase == "full_zero":
                    self.calibration = (axis, now+.6, "full_cogging")
                    self.put("P CALIBRATION COGGING")
                    return
                if phase in ("cogging", "full_cogging"):
                    self.cogging_valid[axis] = True
                if phase == "full_cogging":
                    self.calibrated[axis] = True
                self.calibration = None
                self.put("OK CALIBRATED_FULL" if phase == "full_cogging" else "OK COGGING_CALIBRATED")
            else:
                return
        if self.host and now >= self.next_frame:
            self.next_frame = now + .02
            for axis in (0, 1):
                self.put(f"T {axis} {int(self.enabled[axis])} {int(self.calibrated[axis])} 0 "
                         f"{self.rpm[axis]:.5g} {self.target[axis]:.5g} {self.position[axis]:.5g} "
                         f"12.1 0 {0.15 if self.enabled[axis] else 0}")

    @property
    def in_waiting(self):
        self._pump()
        return len(self.buffer)

    def read(self, size):
        self._pump()
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        if not result:
            time.sleep(.002)
        return result

    def write(self, data):
        args = data.decode("ascii").strip().split()
        if args == ["host", "on"]:
            self.host = True
            self.put("OK HOST 1")
            return len(data)
        try:
            axis = int(args[2] if args[0] in ("param", "cogging") else args[1])
            if axis not in (0, 1):
                raise ValueError
            if args[0] == "cogging":
                if any(self.enabled):
                    self.put("ERR STOP_BOTH_AXES")
                elif not self.calibrated[axis] or self.dirty[axis]:
                    self.put("ERR SAVE_AND_CALIBRATE")
                elif args[1] == "calibrate":
                    self.cogging_valid[axis] = False
                    self.put("P CALIBRATION COGGING")
                    self.calibration = (axis, time.monotonic()+.6, "cogging")
                else:
                    self.put("ERR COMMAND")
            elif args[0] == "tune" and len(args) == 5:
                values = [float(v) for v in args[2:]]
                if any(not math.isfinite(v) or v < 0 for v in values):
                    self.put("ERR PARAMETER")
                else:
                    tuning = dict(zip(("speed_kp", "speed_ki", "cogging_gain"), args[2:]))
                    self.params[axis].update(tuning)
                    self.tuning[axis] = tuning
                    self.tuning_saved[axis] = True
                    self.put("OK")
            elif args[0] == "enable" and len(args) == 3 and args[2] in ("0", "1"):
                want = args[2] == "1"
                if want and (not self.calibrated[axis] or self.dirty[axis]):
                    self.put(f"ERR ENABLE axis={axis} state=0 reason=UNCALIBRATED")
                else:
                    self.enabled[axis] = want
                    if not want:
                        self.target[axis] = 0
                    self.put(f"OK ENABLE axis={axis} state={int(want)}")
            elif args[0] == "speed" and len(args) == 3:
                if not self.enabled[axis]:
                    self.put("ERR STOPPED")
                else:
                    self.target[axis] = float(args[2])
                    self.put("OK")
            elif args[0] == "position":
                self.put("ERR POSITION_NOT_IMPLEMENTED")
            elif args[0] == "calibrate":
                if any(self.enabled) or self.dirty[axis]:
                    self.put("ERR CALIBRATION_ZERO 2")
                else:
                    self.calibrated[axis] = False
                    self.cogging_valid[axis] = False
                    self.put("P CALIBRATION ZERO")
                    self.calibration = (axis, time.monotonic()+.6, "full_zero")
            elif args[:2] == ["param", "show"]:
                self.put(f"OK axis={axis} saved=1 dirty={int(self.dirty[axis])} result=0 running={int(self.enabled[axis])} tuning_saved={int(self.tuning_saved[axis])} tuning_result={0 if self.tuning_saved[axis] else 1}")
                for key, value in self.params[axis].items():
                    self.put(f"{key}={value}")
                self.put(f"calibrated={int(self.calibrated[axis])}")
                self.put("encoder_direction=1")
                self.put("electrical_zero_rad=0.18")
                self.put(f"cogging_valid={int(self.cogging_valid[axis])}")
                self.put("cogging_points=1024")
                self.put("END")
            elif args[:2] == ["param", "set"] and len(args) == 5:
                key, value = args[3:]
                v = float(value)
                if key not in self.params[axis] or key in ("speed_kp", "speed_ki", "cogging_gain") or not math.isfinite(v) or v < 0:
                    self.put("ERR PARAMETER")
                elif key == "current_limit_a" and not 0 < v <= 10:
                    self.put("ERR PARAMETER")
                else:
                    self.params[axis][key] = value
                    self.dirty[axis] = True
                    self.put("OK STAGED")
            elif args[:2] == ["param", "save"]:
                if any(self.enabled):
                    self.put("ERR STOP_BOTH_AXES")
                else:
                    self.persisted[axis] = self.params[axis].copy()
                    self.calibrated[axis] = self.dirty[axis] = False
                    self.cogging_valid[axis] = False
                    self.put("OK")
            elif args[:2] == ["param", "reload"]:
                self.params[axis] = self.persisted[axis].copy()
                if self.tuning[axis] is not None:
                    self.params[axis].update(self.tuning[axis])
                self.dirty[axis] = False
                self.put("OK")
            else:
                self.put("ERR COMMAND")
        except (IndexError, ValueError):
            self.put("ERR COMMAND")
        return len(data)

    def close(self):
        self.host = False

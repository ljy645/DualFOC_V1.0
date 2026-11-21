"""USB CDC 文本协议。串口线程独占读写，GUI 只通过队列交互。"""
from __future__ import annotations

from dataclasses import dataclass, field
import queue
import threading
import time
import math

import serial
from serial.tools import list_ports


@dataclass
class Telemetry:
    axis: int
    enabled: bool
    calibrated: bool
    faults: int
    rpm: float
    target: float
    position: float
    bus: float
    id_a: float
    iq_a: float
    received: float = field(default_factory=time.monotonic)

    @classmethod
    def parse(cls, line: str) -> "Telemetry":
        parts = line.split()
        if len(parts) != 11 or parts[0] != "T":
            raise ValueError("遥测字段数不匹配")
        axis, enabled, calibrated, faults = map(int, parts[1:5])
        if axis not in (0, 1) or enabled not in (0, 1) or calibrated not in (0, 1):
            raise ValueError("遥测轴号或状态无效")
        values = list(map(float, parts[5:]))
        if any(math.isinf(value) for value in values):
            raise ValueError("遥测包含无穷值")
        return cls(axis, bool(enabled), bool(calibrated), faults, *values)


@dataclass
class Job:
    label: str
    commands: list[str]
    axis: int | None = None


def ports() -> list[tuple[str, str]]:
    return [(p.device, p.description) for p in list_ports.comports()]


def parameters(lines: list[str]) -> dict[str, str]:
    """param show 的头部状态与 key=value 行共用同一个解析器。"""
    result = {}
    for line in lines:
        for token in line.split():
            if "=" in token:
                key, value = token.split("=", 1)
                result[key] = value
    return result


class SerialLink(threading.Thread):
    def __init__(self, settings: dict, events: queue.Queue, demo: bool = False):
        super().__init__(name="FOC USB", daemon=True)
        self.settings = settings
        self.events = events
        self.demo = demo
        self.jobs: queue.Queue[Job] = queue.Queue()
        self.stop_event = threading.Event()
        self.device = None
        self.buffer = bytearray()

    def stop(self):
        self.stop_event.set()

    def _write(self, command: str):
        data = (command + "\n").encode("ascii")
        if len(data) > 128 or b"\n" in data[:-1] or b"\r" in data:
            raise ValueError("命令过长或含有额外换行")
        if self.device.write(data) != len(data):
            raise serial.SerialTimeoutException("命令未完整发送")
        self.events.put(("log", "TX", command))

    def _read(self) -> list[str]:
        self.buffer.extend(self.device.read(min(max(self.device.in_waiting, 1), 4096)))
        if len(self.buffer) > 16384:
            raise ValueError("文本帧失步，请确认已烧录配套固件")
        result = []
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer[:] = rest
            try:
                result.append(raw.rstrip(b"\r").decode("ascii"))
            except UnicodeDecodeError as error:
                raise ValueError("收到二进制帧，设备已退出 HOST 文本模式") from error
        return result

    def _telemetry(self, line: str) -> bool:
        if line.startswith("T "):
            self.events.put(("telemetry", Telemetry.parse(line)))
            return True
        return False

    def _request(self, command: str) -> list[str]:
        self._write(command)
        deadline = time.monotonic() + (120.0 if command.startswith(("cogging calibrate ", "calibrate ")) else 3.0)
        multi = command.startswith("param show ")
        reply = []
        complete = False
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            # 一次 read 可能带回回复和随后的遥测，全部处理，不能丢掉尾部帧。
            for line in self._read():
                if self._telemetry(line):
                    continue
                if not line:
                    continue
                self.events.put(("log", "RX", line))
                if line in ("P CALIBRATION ZERO", "P CALIBRATION COGGING"):
                    self.events.put(("phase", line.split()[-1]))
                    continue
                if complete:
                    raise ValueError("收到未配对的应答：" + line)
                reply.append(line)
                if line.startswith("ERR "):
                    complete = True
                elif multi:
                    complete = line == "END"
                else:
                    complete = line.startswith("OK")
            if complete:
                if command.startswith("calibrate ") and reply[0].startswith("OK") and reply[0] != "OK CALIBRATED_FULL":
                    return ["ERR FULL_CALIBRATION_UNSUPPORTED"]
                return reply
        if self.stop_event.is_set():
            raise InterruptedError("连接已关闭")
        raise TimeoutError(f"{command} 等待应答超时；不重发，断开后请重新连接查询状态")

    def _handshake(self):
        self._write("host on")
        raw = bytearray()
        marker = b"OK HOST 1\r\n"
        deadline = time.monotonic() + 3.0
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            raw.extend(self.device.read(min(max(self.device.in_waiting, 1), 4096)))
            at = raw.find(marker)
            if at >= 0:
                self.buffer[:] = raw[at + len(marker):]
                self.events.put(("log", "RX", "OK HOST 1"))
                return
            # 握手前允许在途的 JustFloat 帧，正常文本接收后不再容忍二进制。
            if len(raw) > 16384:
                del raw[:-len(marker)]
        raise TimeoutError("HOST 握手失败，请烧录配套固件；已连接串口不代表协议已就绪")

    def run(self):
        try:
            if self.demo:
                from demo_device import DemoSerial
                self.device = DemoSerial()
            else:
                self.device = serial.Serial(**self.settings, timeout=0.02, write_timeout=0.5)
            self._handshake()
            self.events.put(("connected", self.demo))
            while not self.stop_event.is_set():
                try:
                    job = self.jobs.get_nowait()
                except queue.Empty:
                    for line in self._read():
                        if line and not self._telemetry(line):
                            self.events.put(("log", "RX", line))
                    continue
                replies = []
                success = True
                for command in job.commands:
                    answer = self._request(command)
                    replies.append((command, answer))
                    if answer and answer[0].startswith("ERR "):
                        success = False
                        break
                self.events.put(("done", job, success, replies))
        except InterruptedError:
            pass
        except (serial.SerialException, OSError, ValueError, TimeoutError) as error:
            self.events.put(("error", str(error)))
        finally:
            if self.device is not None:
                # 断开只释放端口，不隐式使能、停机或重放任何运动命令。
                self.device.close()
            self.events.put(("disconnected",))

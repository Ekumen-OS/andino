#!/usr/bin/env python3
# BSD 3-Clause License
#
# Copyright (c) 2026, Ekumen Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Validates the hardware connections to the MCU and the firmware flashing.

Run it from the `andino_firmware` directory, with the board connected through USB:

  docker compose -f docker/compose.yaml run --rm dev python tools/hw_check.py --port /dev/ttyUSB0

It runs all the checks, which only rely on the firmware serial protocol:

  * Firmware: the MCU answers, the firmware booted correctly and its version is reported.
  * IMU: the sensor is detected and reports sane values at rest.
  * Encoders: the tick counts are stable at rest, and both channels of each encoder work when its
    wheel is rotated by hand.
  * Motors: each motor moves the right wheel, in the right direction, and its encoder counts it.
  * Closed loop: the speed control reaches the requested speed.

Whatever the tool can't validate on its own, like whether a wheel actually spins in the expected
direction, is asked to the user as a yes/no question. The answers are combined with the encoder
readings to tell apart, e.g., a motor problem from an encoder problem.

The exit code is 0 if no check failed, 1 if any check failed (including an MCU that doesn't
answer) and 2 if the serial port can't be found or opened.
"""

import argparse
import dataclasses
import math
import re
import sys
import time
from typing import Callable, Iterable, List, Optional, Tuple

PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"
SKIP = "SKIP"

EXIT_OK = 0
EXIT_CHECKS_FAILED = 1
EXIT_ERROR = 2

BAUD_RATE = 57600
ERROR_PREFIX = "[ERROR]"

# A firmware version, e.g. "0.2.0".
VERSION_PATTERN = re.compile(r"[\w.+-]+")

# USB vendor IDs of the usual Arduino boards and USB-to-serial chips (Arduino, WCH CH340, FTDI,
# Silicon Labs CP210x).
KNOWN_USB_VENDOR_IDS = (0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4)

# Wheel names, by their index in the firmware commands.
WHEELS = ("left", "right")

# Time to wait after opening the serial port, which resets most Arduino boards [s].
BOOT_DELAY_S = 2.0
# Time to wait for the MCU to reply to a command [s].
REPLY_TIMEOUT_S = 1.0


class McuError(Exception):
    """The MCU replied with an error or with an unexpected message."""


class McuTimeout(McuError):
    """The MCU did not reply in time."""


@dataclasses.dataclass
class Result:
    """Outcome of a check."""

    name: str
    status: str
    detail: str = ""
    hint: str = ""


@dataclasses.dataclass
class Options:
    """Checks configuration."""

    # Number of attempts to get an answer from the MCU.
    retries: int = 3
    # Minimum number of ticks that an encoder must count to consider that a wheel moved.
    min_ticks: int = 5
    # Time the user has to rotate each wheel by hand [s].
    hand_rotate_s: float = 5.0
    # PWM value and duration of the motor pulses [s].
    pwm: int = 120
    pulse_s: float = 0.7
    # Wait before the motors start moving, so the user can watch them [s].
    warning_s: float = 1.5
    # Closed-loop target speed [ticks/s] and accepted relative error.
    speed_tps: int = 300
    speed_tolerance: float = 0.3


class Mcu:
    """Client of the firmware serial protocol, on top of a pyserial-like port."""

    def __init__(self, port):
        self._port = port

    def drain(self) -> str:
        """Discards (and returns) anything the MCU has sent, e.g. its boot messages."""
        data = b""
        while self._port.in_waiting:
            data += self._port.read(self._port.in_waiting)
        return data.decode("ascii", errors="replace")

    def send(self, command: str) -> str:
        """Sends a command and returns the one-line reply, without its terminator."""
        self._port.write(f"{command}\r".encode("ascii"))
        line = self._port.readline()
        if not line.endswith(b"\n"):
            raise McuTimeout(f"No reply to '{command}'")
        return line.decode("ascii", errors="replace").strip()

    def query(self, command: str) -> str:
        """Sends a command and returns its reply, raising if the MCU replies with an error."""
        reply = self.send(command)
        if reply.startswith(ERROR_PREFIX):
            raise McuError(f"'{command}' failed: {reply}")
        return reply

    def execute(self, command: str) -> None:
        """Sends a write command, which must be acknowledged."""
        reply = self.query(command)
        if reply != "[OK]":
            raise McuError(f"Unexpected reply to '{command}': {reply}")

    def read_encoders(self) -> Tuple[int, int]:
        values = self.query("getenc").split()
        if len(values) != 2:
            raise McuError(f"Unexpected reply to 'getenc': {values}")
        return int(values[0]), int(values[1])

    def read_channel(self, encoder: int, channel: int) -> int:
        return int(self.query(f"getch {encoder} {channel}"))

    def stop_motors(self) -> None:
        self.execute("setpwm 0 0")


def ask_yes_no(question: str) -> bool:
    """Asks the user a yes/no question. Without an answer (e.g. no terminal), the answer is no."""
    while True:
        try:
            answer = input(f"{question} (yes/no): ").strip().lower()
        except EOFError:
            print()
            return False
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please answer 'yes' or 'no'.")


def diagnose_motor(
    name: str, sign: int, own: int, other: int, saw_motion: bool, min_ticks: int
) -> Result:
    """Combines the encoder readings and what the user saw to tell what is wrong with a motor.

    Args:
        name: Name of the check.
        sign: Commanded direction, 1 for forward and -1 for backward.
        own: Ticks counted by the encoder of the commanded wheel.
        other: Ticks counted by the encoder of the other wheel.
        saw_motion: Whether the user saw the commanded wheel spin in the commanded direction.
        min_ticks: Minimum number of ticks to consider that an encoder counted.
    """
    counted = abs(own) >= min_ticks
    other_counted = abs(other) >= min_ticks
    right_way = counted and own * sign > 0

    if saw_motion:
        if right_way and not other_counted:
            return Result(name, PASS, f"The wheel spun as expected and {own} ticks were counted.")
        if right_way:
            return Result(
                name,
                FAIL,
                f"The wheel spun as expected, but the other encoder also counted {other} ticks.",
                "The encoders may be cross-wired.",
            )
        if counted:
            return Result(
                name,
                FAIL,
                f"The wheel spun as expected, but its encoder counted backwards ({own} ticks).",
                "The encoder channels A and B are reversed.",
            )
        if other_counted:
            return Result(
                name,
                FAIL,
                f"The wheel spun as expected, but the other encoder counted {other} ticks.",
                "The left and right encoders are swapped.",
            )
        return Result(
            name,
            FAIL,
            "The wheel spun as expected, but its encoder didn't count.",
            "Check the encoder power and wiring.",
        )

    if right_way:
        return Result(
            name,
            FAIL,
            f"The encoder counted {own} ticks, but the wheel didn't spin as expected.",
            "Both the motor polarity and the encoder channels are reversed, or the encoder "
            "is picking up noise.",
        )
    if counted:
        return Result(
            name,
            FAIL,
            f"The wheel spun the wrong way ({own} ticks counted).",
            "The motor polarity is reversed: swap the motor wires.",
        )
    if other_counted:
        return Result(
            name,
            FAIL,
            f"The wheel didn't spin, but the other encoder counted {other} ticks.",
            "The left and right motors are swapped.",
        )
    direction = "forward" if sign > 0 else "backward"
    return Result(
        name,
        FAIL,
        f"The wheel didn't spin {direction} and no ticks were counted.",
        "Check the motor power supply, the motor driver and the motor wiring. If the wheel "
        "spun the other way, the motor polarity is also reversed.",
    )


class Checker:
    """Runs the hardware checks."""

    def __init__(
        self,
        mcu: Mcu,
        options: Optional[Options] = None,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
        ask: Callable[[str], bool] = ask_yes_no,
        say: Callable[[str], None] = print,
    ):
        self._mcu = mcu
        self._options = options or Options()
        self._sleep = sleep
        self._clock = clock
        self._ask = ask
        self._say = say

    def run(self, on_result: Callable[[Result], None] = lambda result: None) -> List[Result]:
        """Runs all the checks, reporting each result as soon as it is available."""
        results: List[Result] = []

        def report(new_results: Iterable[Result]) -> None:
            # The checks report their results as they go, so each one is shown right after the
            # question that led to it.
            for result in new_results:
                results.append(result)
                on_result(result)

        try:
            report(self.check_firmware())
            if any(r.status == FAIL for r in results):
                return results
            report(self.check_imu())
            report(self.check_encoders_at_rest())
            report(self.check_encoders_by_hand())
            if self._wheels_are_free():
                report(self.check_motors())
                report(self.check_closed_loop())
            else:
                reason = "The wheels are not free to spin."
                report([Result("Motors", SKIP, reason), Result("Closed loop", SKIP, reason)])
        except McuError as error:
            report([Result("MCU communication", FAIL, str(error), "Check the cable and the port.")])
        finally:
            self._safe_stop()
        return results

    def _safe_stop(self) -> None:
        try:
            self._mcu.stop_motors()
        except McuError:
            pass

    def _wheels_are_free(self) -> bool:
        self._say("\nThe next checks spin the motors.")
        return self._ask("Are the wheels lifted off the ground (or free to spin)?")

    def check_firmware(self) -> List[Result]:
        results: List[Result] = []

        boot_output = self._mcu.drain()
        if "Failed to register commands" in boot_output:
            return [
                Result(
                    "Firmware boot",
                    FAIL,
                    "The firmware failed to register its commands.",
                    "Flash a build of the firmware without the command registration problem.",
                )
            ]

        reply = None
        for _ in range(self._options.retries):
            try:
                reply = self._mcu.send("ver")
                break
            except McuTimeout:
                continue
        if reply is None:
            return [
                Result(
                    "MCU response",
                    FAIL,
                    "The MCU does not answer.",
                    f"Check the port, the cable, the baud rate ({BAUD_RATE}) and that the "
                    "firmware is flashed.",
                )
            ]
        results.append(Result("MCU response", PASS, "The MCU answers commands."))

        if reply.startswith(ERROR_PREFIX) or not VERSION_PATTERN.fullmatch(reply):
            results.append(
                Result(
                    "Firmware version",
                    WARN,
                    f"The firmware doesn't support the 'ver' command (it replied '{reply}').",
                    "Flash a recent version of the firmware.",
                )
            )
        else:
            results.append(Result("Firmware version", PASS, reply))

        unknown = self._mcu.send("hw_check_unknown_command")
        if unknown == f"{ERROR_PREFIX} Unknown command":
            results.append(Result("Protocol", PASS, "Errors are reported as expected."))
        else:
            results.append(
                Result(
                    "Protocol",
                    WARN,
                    f"Unexpected reply to an unknown command: '{unknown}'.",
                    "The firmware may not use the current serial protocol.",
                )
            )
        return results

    def check_imu(self) -> List[Result]:
        name = "IMU"
        if self._mcu.query("hasimu") != "1":
            if self._ask("The IMU was not detected. Does this robot have an IMU connected?"):
                return [Result(name, FAIL, "The IMU was not detected.", "Check the I2C wiring.")]
            return [Result(name, SKIP, "The robot has no IMU.")]

        values = [float(value) for value in self._mcu.query("getencimu").split()]
        if len(values) != 12:
            return [Result(name, FAIL, f"Unexpected 'getencimu' reply: {values}")]
        orientation, angular_velocity, linear_acceleration = values[2:6], values[6:9], values[9:12]

        quaternion_norm = math.sqrt(sum(v * v for v in orientation))
        problems = []
        if not 0.95 <= quaternion_norm <= 1.05:
            problems.append(f"orientation is not a unit quaternion (norm {quaternion_norm:.2f})")
        if max(abs(v) for v in angular_velocity) > 0.5:
            problems.append("angular velocity is not ~0 at rest (is the robot still?)")
        if max(abs(v) for v in linear_acceleration) > 1.0:
            problems.append("linear acceleration is not ~0 at rest (is the robot still?)")
        if problems:
            return [Result(name, FAIL, "; ".join(problems), "Keep the robot still and retry.")]
        return [Result(name, PASS, "Detected, with sane values at rest.")]

    def check_encoders_at_rest(self) -> List[Result]:
        first = self._mcu.read_encoders()
        self._sleep(0.3)
        second = self._mcu.read_encoders()
        if first != second:
            return [
                Result(
                    "Encoders at rest",
                    WARN,
                    f"The tick count changed without any motion: {first} -> {second}.",
                    "Check the encoder wiring for noise or a loose connection.",
                )
            ]
        return [Result("Encoders at rest", PASS, f"Readable and stable at {first}.")]

    def check_encoders_by_hand(self) -> Iterable[Result]:
        for index, wheel in enumerate(WHEELS):
            name = f"Encoder, {wheel} (by hand)"
            self._say(f"\nNext, rotate the {wheel.upper()} wheel by hand.")
            if not self._ask(
                f"Are you ready to rotate it for {self._options.hand_rotate_s:.0f} s, "
                "starting as soon as you answer?"
            ):
                yield Result(name, SKIP, "Skipped by the user.")
                continue
            self._mcu.execute("rstenc")
            levels = {0: set(), 1: set()}
            end = self._clock() + self._options.hand_rotate_s
            while self._clock() < end:
                for channel in (0, 1):
                    levels[channel].add(self._mcu.read_channel(index, channel))
            counts = self._mcu.read_encoders()
            yield self._evaluate_hand_rotation(name, index, counts, levels)

    def _evaluate_hand_rotation(self, name, index, counts, levels) -> Result:
        problems = []
        for channel, label in enumerate(("A", "B")):
            if levels[channel] != {0, 1}:
                problems.append(f"channel {label} never changed")
        if abs(counts[index]) < self._options.min_ticks:
            problems.append(f"only {counts[index]} ticks counted")
        other = counts[1 - index]
        if abs(other) >= self._options.min_ticks:
            problems.append(f"the {WHEELS[1 - index]} encoder counted {other} ticks instead")
        if problems:
            return Result(
                name,
                FAIL,
                "; ".join(problems) + ".",
                "Check the encoder power, channel wiring and that the left/right encoders "
                "are not swapped.",
            )
        return Result(name, PASS, f"Both channels work, {counts[index]} ticks counted.")

    def check_motors(self) -> Iterable[Result]:
        for index, wheel in enumerate(WHEELS):
            for sign, direction in ((1, "forward"), (-1, "backward")):
                yield self._check_motor_pulse(index, wheel, sign, direction)

    def _check_motor_pulse(self, index, wheel, sign, direction) -> Result:
        pwm = [0, 0]
        pwm[index] = sign * self._options.pwm
        self._say(f"\nThe {wheel.upper()} wheel is going to spin {direction.upper()}. Watch it.")
        self._sleep(self._options.warning_s)
        try:
            self._mcu.execute("rstenc")
            self._mcu.execute(f"setpwm {pwm[0]} {pwm[1]}")
            self._sleep(self._options.pulse_s)
            self._mcu.stop_motors()
            counts = self._mcu.read_encoders()
        finally:
            self._safe_stop()

        saw_motion = self._ask(f"Did the {wheel.upper()} wheel spin {direction.upper()}?")
        return diagnose_motor(
            f"Motor, {wheel} {direction}",
            sign,
            counts[index],
            counts[1 - index],
            saw_motion,
            self._options.min_ticks,
        )

    def _closed_loop_hint(self, rate: float) -> str:
        """Tells what to check when the wheel doesn't reach the target speed."""
        if abs(rate) < self._options.min_ticks:
            return "The wheel didn't move or its encoder didn't count: see the motor checks."
        if rate < 0:
            return "The encoder counted backwards: see the motor checks."
        return "Check the PID gains (setpid), the motor supply and the speed target."

    def check_closed_loop(self) -> Iterable[Result]:
        target = self._options.speed_tps
        self._say("\nBoth wheels are going to spin FORWARD at a steady speed. Watch them.")
        self._sleep(self._options.warning_s)
        try:
            self._mcu.execute("rstenc")
            self._mcu.execute(f"setspd {target} {target}")
            self._sleep(0.5)
            first, start = self._mcu.read_encoders(), self._clock()
            self._sleep(1.0)
            second, end = self._mcu.read_encoders(), self._clock()
        finally:
            self._safe_stop()

        smooth = self._ask(
            "Did both wheels spin at a steady speed, without stuttering or oscillating?"
        )

        for index, wheel in enumerate(WHEELS):
            rate = (second[index] - first[index]) / (end - start)
            name = f"Closed loop, {wheel}"
            detail = f"{rate:.0f} ticks/s for a target of {target}."
            if abs(rate - target) <= self._options.speed_tolerance * target:
                yield Result(name, PASS, detail)
            else:
                yield Result(name, FAIL, detail, self._closed_loop_hint(rate))

        name = "Closed loop, smoothness"
        if smooth:
            yield Result(name, PASS, "Steady speed.")
        else:
            yield Result(
                name, FAIL, "The wheels stuttered or oscillated.", "Tune the PID gains (setpid)."
            )


def find_port(list_ports=None) -> Optional[str]:
    """Returns the serial port of the only Arduino-like device connected, if there is one."""
    if list_ports is None:
        from serial.tools import list_ports as serial_list_ports

        list_ports = serial_list_ports.comports
    candidates = [p.device for p in list_ports() if p.vid in KNOWN_USB_VENDOR_IDS]
    return candidates[0] if len(candidates) == 1 else None


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validates the hardware connections to the MCU and the firmware flashing. "
        "It runs all the checks and asks yes/no questions about what it can't validate on "
        "its own."
    )
    parser.add_argument(
        "--port", help="serial port of the MCU (auto-detected if there is only one device)"
    )
    return parser.parse_args(argv)


def print_result(result: Result) -> None:
    print(f"[{result.status}] {result.name}" + (f": {result.detail}" if result.detail else ""))
    if result.hint and result.status in (FAIL, WARN):
        print(f"       -> {result.hint}")


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    import serial  # Imported here so the checks can be unit tested without pyserial.

    port_name = args.port or find_port()
    if port_name is None:
        print("No single Arduino-like device found: pass the serial port with --port.")
        return EXIT_ERROR
    print(f"Using {port_name} at {BAUD_RATE} baud.")

    try:
        port = serial.Serial(port_name, BAUD_RATE, timeout=REPLY_TIMEOUT_S)
    except serial.SerialException as error:
        print(f"Unable to open {port_name}: {error}")
        return EXIT_ERROR

    with port:
        time.sleep(BOOT_DELAY_S)  # Opening the port resets most Arduino boards.
        results = Checker(Mcu(port)).run(print_result)

    failed = [r for r in results if r.status == FAIL]
    print(f"\n{len(results)} checks, {len(failed)} failed.")
    return EXIT_CHECKS_FAILED if failed else EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

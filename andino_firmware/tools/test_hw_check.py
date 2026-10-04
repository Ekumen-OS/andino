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

"""Unit tests of hw_check.py, run against a simulated robot and a simulated user."""

import unittest
from unittest import mock

import hw_check
from hw_check import FAIL, PASS, SKIP, WARN

TICKS_PER_PWM = 4.0  # Simulated wheel speed [ticks/s] per PWM unit.
HAND_SPEED = 20.0  # Simulated wheel speed when rotated by hand [ticks/s].


class FakeRobot:
    """A serial port connected to a simulated Andino robot (firmware, motors and encoders).

    The simulation keeps the physical motion of the wheels apart from what the encoders report, so
    faults like reversed encoders can be told apart from reversed motors. Every command takes 10 ms
    of simulated time, and the simulated time only moves when commands are sent or when sleep() is
    called, so tests run instantly.
    """

    def __init__(self, **faults):
        self.faults = faults
        self.now = 0.0
        self.pwm = [0, 0]
        self.target_speed = None
        self.hand_wheel = None
        self.hand_until = 0.0
        # Net physical rotation of each wheel since the last motor command [ticks].
        self.physical = [0.0, 0.0]
        # Encoder counts of each wheel.
        self.ticks = [0.0, 0.0]
        self._rx = faults.get("boot_output", "").encode()
        self.commands = []

    # pyserial-like interface.
    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, size):
        data, self._rx = self._rx[:size], self._rx[size:]
        return data

    def readline(self):
        if b"\n" in self._rx:
            line, _, self._rx = self._rx.partition(b"\n")
            return line + b"\n"
        self.advance(1.0)
        return b""

    def write(self, data):
        command = data.decode().strip("\r")
        self.commands.append(command)
        self.advance(0.01)
        reply = self._execute(command)
        if reply is not None:
            self._rx += (reply + "\n").encode()

    # Simulation.
    def clock(self):
        return self.now

    def advance(self, seconds):
        for wheel in range(2):
            rate = self._physical_rate(wheel)
            self.physical[wheel] += rate * seconds
            encoder_sign = -1 if wheel in self.faults.get("reversed_encoders", ()) else 1
            self.ticks[wheel] += rate * encoder_sign * seconds
        self.now += seconds

    def rotate_by_hand(self, wheel, seconds):
        self.hand_wheel, self.hand_until = wheel, self.now + seconds

    def _physical_rate(self, wheel):
        if self.hand_wheel == wheel and self.now < self.hand_until:
            return HAND_SPEED
        motor = 1 - wheel if self.faults.get("swapped_motors") else wheel
        if motor in self.faults.get("dead_motors", ()):
            return 0.0
        if self.target_speed is not None:
            rate = self.target_speed[motor] * self.faults.get("speed_factor", 1.0)
        else:
            rate = self.pwm[motor] * TICKS_PER_PWM
        return -rate if self.faults.get("reversed_motors") else rate

    def _count(self, wheel):
        if wheel in self.faults.get("dead_encoders", ()):
            return 0
        source = 1 - wheel if self.faults.get("swapped_encoders") else wheel
        return int(self.ticks[source])

    def _level(self, wheel, channel):
        if (wheel, channel) in self.faults.get("stuck_channels", ()):
            return 1
        count = self._count(wheel)
        return (count // 2) % 2 if channel == 0 else ((count + 1) // 2) % 2

    def _start_motion(self):
        self.physical = [0.0, 0.0]

    def _execute(self, command):
        name, *args = command.split()
        if "no_reply" in self.faults:
            return None
        if "fail_command" in self.faults and name == self.faults["fail_command"]:
            return None
        if name == "ver":
            if "old_firmware" in self.faults:
                return "Unknown command."
            return self.faults.get("version", "0.2.0")
        if name == "hasimu":
            return "0" if "no_imu" in self.faults else "1"
        if name == "getencimu":
            quaternion = self.faults.get("quaternion", "0.0000 0.0000 0.7071 0.7071")
            return f"{self._count(0)} {self._count(1)} {quaternion} 0.01 0.00 -0.01 0.05 0.02 0.10"
        if name == "getenc":
            return f"{self._count(0)} {self._count(1)}"
        if name == "getch":
            return str(self._level(int(args[0]), int(args[1])))
        if name == "rstenc":
            self.ticks = [0.0, 0.0]
            return "[OK]"
        if name == "setpwm":
            self.pwm = [int(args[0]), int(args[1])]
            self.target_speed = None
            if any(self.pwm):
                self._start_motion()
            return "[OK]"
        if name == "setspd":
            self.target_speed = [int(args[0]), int(args[1])]
            self._start_motion()
            return "[OK]"
        if "old_firmware" in self.faults:
            return "Unknown command."
        return "[ERROR] Unknown command"


class FakeUser:
    """A simulated user, who answers the questions by looking at the simulated robot."""

    def __init__(self, robot, has_imu=True, wheels_free=True, ready=True, smooth=True):
        self.robot = robot
        self.has_imu = has_imu
        self.wheels_free = wheels_free
        self.ready = ready
        self.smooth = smooth
        self.questions = []
        self._wheel_to_rotate = None

    def say(self, message):
        # The wheel to rotate by hand is announced before asking if the user is ready.
        if "rotate the" in message:
            self._wheel_to_rotate = 0 if "LEFT" in message else 1

    def ask(self, question):
        self.questions.append(question)
        if "IMU" in question:
            return self.has_imu
        if "free to spin" in question:
            return self.wheels_free
        if "ready to rotate" in question:
            if self.ready:
                self.robot.rotate_by_hand(self._wheel_to_rotate, hw_check.Options().hand_rotate_s)
            return self.ready
        if "steady speed" in question:
            return self.smooth
        if "Did the" in question:
            wheel = 0 if "LEFT" in question else 1
            expected = 1 if "FORWARD" in question else -1
            return self.robot.physical[wheel] * expected > hw_check.Options().min_ticks
        raise AssertionError(f"Unexpected question: {question}")


def run_checks(robot, user=None, **options):
    user = user or FakeUser(robot)
    checker = hw_check.Checker(
        hw_check.Mcu(robot),
        hw_check.Options(**options),
        sleep=robot.advance,
        clock=robot.clock,
        ask=user.ask,
        say=user.say,
    )
    return {result.name: result for result in checker.run()}, user


class HwCheckTest(unittest.TestCase):
    def assert_status(self, results, name, status):
        self.assertIn(name, results)
        self.assertEqual(results[name].status, status, results[name])

    def test_healthy_robot_passes_all_checks(self):
        robot = FakeRobot()
        results, user = run_checks(robot)

        self.assertEqual([r.name for r in results.values() if r.status != PASS], [])
        self.assertEqual(len(results), 14)
        self.assertEqual(robot.pwm, [0, 0])
        self.assertEqual(robot.commands[-1], "setpwm 0 0")
        self.assertEqual(sum("Did the" in q for q in user.questions), 4)

    def test_no_reply_fails(self):
        results, _ = run_checks(FakeRobot(no_reply=True))

        self.assert_status(results, "MCU response", FAIL)
        self.assertEqual(len(results), 1)

    def test_boot_error_fails(self):
        results, _ = run_checks(FakeRobot(boot_output="[ERROR] Failed to register commands\n"))

        self.assert_status(results, "Firmware boot", FAIL)

    def test_old_firmware_warns(self):
        results, _ = run_checks(FakeRobot(old_firmware=True))

        self.assert_status(results, "Firmware version", WARN)
        self.assert_status(results, "Protocol", WARN)

    def test_firmware_version_is_reported(self):
        results, _ = run_checks(FakeRobot(version="1.2.3"))

        self.assertEqual(results["Firmware version"].detail, "1.2.3")

    def test_missing_imu_is_skipped_if_the_robot_has_none(self):
        robot = FakeRobot(no_imu=True)
        results, _ = run_checks(robot, FakeUser(robot, has_imu=False))

        self.assert_status(results, "IMU", SKIP)

    def test_missing_imu_fails_if_the_robot_has_one(self):
        robot = FakeRobot(no_imu=True)
        results, _ = run_checks(robot, FakeUser(robot, has_imu=True))

        self.assert_status(results, "IMU", FAIL)

    def test_invalid_imu_orientation_fails(self):
        results, _ = run_checks(FakeRobot(quaternion="0.0 0.0 0.0 0.0"))

        self.assert_status(results, "IMU", FAIL)

    def test_encoder_counting_at_rest_warns(self):
        robot = FakeRobot()
        robot.rotate_by_hand(0, float("inf"))  # Moving without any command.

        results, _ = run_checks(robot)

        self.assert_status(results, "Encoders at rest", WARN)

    def test_stuck_encoder_channel_fails(self):
        results, _ = run_checks(FakeRobot(stuck_channels=((1, 1),)))

        self.assert_status(results, "Encoder, right (by hand)", FAIL)
        self.assertIn("channel B never changed", results["Encoder, right (by hand)"].detail)
        self.assert_status(results, "Encoder, left (by hand)", PASS)

    def test_encoders_by_hand_can_be_skipped(self):
        robot = FakeRobot()
        results, _ = run_checks(robot, FakeUser(robot, ready=False))

        self.assert_status(results, "Encoder, left (by hand)", SKIP)
        self.assert_status(results, "Encoder, right (by hand)", SKIP)

    def test_motors_are_not_driven_if_the_wheels_are_not_free(self):
        robot = FakeRobot()
        results, _ = run_checks(robot, FakeUser(robot, wheels_free=False))

        self.assert_status(results, "Motors", SKIP)
        self.assert_status(results, "Closed loop", SKIP)
        self.assertFalse([c for c in robot.commands if c.startswith(("setspd", "setpwm 1"))])

    def test_swapped_motors_fail(self):
        results, _ = run_checks(FakeRobot(swapped_motors=True))

        self.assert_status(results, "Motor, left forward", FAIL)
        self.assertIn("motors are swapped", results["Motor, left forward"].hint)

    def test_reversed_motor_fails(self):
        results, _ = run_checks(FakeRobot(reversed_motors=True))

        self.assert_status(results, "Motor, right backward", FAIL)
        self.assertIn("motor polarity is reversed", results["Motor, right backward"].hint)

    def test_reversed_encoder_fails(self):
        results, _ = run_checks(FakeRobot(reversed_encoders=(0,)))

        self.assert_status(results, "Motor, left forward", FAIL)
        self.assertIn("channels A and B are reversed", results["Motor, left forward"].hint)
        self.assert_status(results, "Motor, right forward", PASS)

    def test_swapped_encoders_fail(self):
        results, _ = run_checks(FakeRobot(swapped_encoders=True))

        self.assert_status(results, "Motor, left forward", FAIL)
        self.assertIn("encoders are swapped", results["Motor, left forward"].hint)

    def test_dead_encoder_is_told_apart_from_a_dead_motor(self):
        results, _ = run_checks(FakeRobot(dead_encoders=(0,)))

        self.assert_status(results, "Motor, left forward", FAIL)
        self.assertIn("encoder didn't count", results["Motor, left forward"].detail)
        self.assert_status(results, "Encoder, left (by hand)", FAIL)
        self.assert_status(results, "Motor, right forward", PASS)

    def test_dead_motor_is_told_apart_from_a_dead_encoder(self):
        results, _ = run_checks(FakeRobot(dead_motors=(1,)))

        self.assert_status(results, "Motor, right forward", FAIL)
        self.assertIn("no ticks were counted", results["Motor, right forward"].detail)
        self.assert_status(results, "Encoder, right (by hand)", PASS)

    def test_slow_closed_loop_points_to_the_pid_and_the_motor_supply(self):
        results, _ = run_checks(FakeRobot(speed_factor=0.5))

        self.assert_status(results, "Closed loop, left", FAIL)
        self.assertIn("PID gains", results["Closed loop, left"].hint)

    def test_closed_loop_with_a_reversed_encoder_points_to_the_motor_checks(self):
        results, _ = run_checks(FakeRobot(reversed_encoders=(0,)))

        self.assert_status(results, "Closed loop, left", FAIL)
        self.assertIn("counted backwards", results["Closed loop, left"].hint)
        self.assertNotIn("PID", results["Closed loop, left"].hint)
        self.assert_status(results, "Closed loop, right", PASS)

    def test_closed_loop_with_a_wheel_that_does_not_move_points_to_the_motor_checks(self):
        results, _ = run_checks(FakeRobot(dead_motors=(1,)))

        self.assert_status(results, "Closed loop, right", FAIL)
        self.assertIn("didn't move", results["Closed loop, right"].hint)
        self.assertNotIn("PID", results["Closed loop, right"].hint)

    def test_unsteady_closed_loop_fails(self):
        robot = FakeRobot()
        results, _ = run_checks(robot, FakeUser(robot, smooth=False))

        self.assert_status(results, "Closed loop, smoothness", FAIL)
        self.assert_status(results, "Closed loop, left", PASS)

    def test_motors_are_stopped_when_a_check_fails_midway(self):
        robot = FakeRobot(fail_command="getenc")
        results, _ = run_checks(robot)

        self.assert_status(results, "MCU communication", FAIL)
        self.assertEqual(robot.pwm, [0, 0])
        self.assertEqual(robot.commands[-1], "setpwm 0 0")

    def test_find_port(self):
        class Port:
            def __init__(self, device, vid):
                self.device, self.vid = device, vid

        arduino, other = Port("/dev/ttyUSB0", 0x1A86), Port("/dev/ttyS0", None)
        self.assertEqual(hw_check.find_port(lambda: [other, arduino]), "/dev/ttyUSB0")
        self.assertIsNone(hw_check.find_port(lambda: [other]))
        self.assertIsNone(hw_check.find_port(lambda: [arduino, Port("/dev/ttyACM0", 0x2341)]))


class DiagnoseMotorTest(unittest.TestCase):
    def diagnose(self, own, other, saw_motion, sign=1):
        return hw_check.diagnose_motor("motor", sign, own, other, saw_motion, 5)

    def test_expected_behavior_passes(self):
        self.assertEqual(self.diagnose(100, 0, True).status, PASS)
        self.assertEqual(self.diagnose(-100, 0, True, sign=-1).status, PASS)

    def test_every_other_combination_fails_with_a_hint(self):
        for own in (100, -100, 0):
            for other in (100, 0):
                for saw_motion in (True, False):
                    if (own, other, saw_motion) == (100, 0, True):
                        continue
                    result = self.diagnose(own, other, saw_motion)
                    self.assertEqual(result.status, FAIL, (own, other, saw_motion))
                    self.assertTrue(result.hint, (own, other, saw_motion))


class AskYesNoTest(unittest.TestCase):
    def ask(self, answers):
        with mock.patch("builtins.input", side_effect=answers):
            return hw_check.ask_yes_no("Question?")

    def test_accepts_yes_and_no_in_any_case(self):
        self.assertTrue(self.ask(["yes"]))
        self.assertTrue(self.ask(["Y"]))
        self.assertFalse(self.ask(["NO"]))
        self.assertFalse(self.ask(["n"]))

    def test_asks_again_until_the_answer_is_valid(self):
        self.assertTrue(self.ask(["maybe", "", "yes"]))

    def test_no_answer_means_no(self):
        self.assertFalse(self.ask(EOFError))


if __name__ == "__main__":
    unittest.main()

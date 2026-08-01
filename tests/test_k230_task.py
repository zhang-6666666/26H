"""Desktop tests for the K230 key mission and extended UART frame."""

import unittest
from unittest import mock
import sys
import types

import main


class BallMissionTests(unittest.TestCase):
    def test_button_runs_positive_then_negative_and_holds(self):
        mission = main.BallMission()
        self.assertEqual(mission.state, mission.IDLE)
        self.assertEqual(mission.target_cm, 0.0)

        self.assertTrue(mission.handle_button(True))
        self.assertEqual(mission.state, mission.GO_POSITIVE)
        self.assertEqual(mission.target_cm, 5.0)
        self.assertFalse(mission.handle_button(True))

        self.assertFalse(mission.update_detection(None))
        self.assertFalse(mission.update_detection({"x_est_cm": 4.69}))
        self.assertTrue(mission.update_detection({"x_est_cm": 4.70}))
        self.assertEqual(mission.state, mission.GO_NEGATIVE)
        self.assertEqual(mission.target_cm, -5.0)

        self.assertTrue(mission.update_detection({"x_est_cm": -4.70}))
        self.assertEqual(mission.state, mission.HOLD_NEGATIVE)
        self.assertEqual(mission.target_cm, -5.0)

        mission.handle_button(False)
        self.assertTrue(mission.handle_button(True))
        self.assertEqual(mission.state, mission.GO_POSITIVE)
        self.assertEqual(mission.target_cm, 5.0)

    def test_active_low_key_debounce(self):
        key = main.ActiveLowKeyDebouncer(debounce_ms=25)
        self.assertFalse(key.update(1, 0))
        self.assertFalse(key.update(0, 10))
        self.assertFalse(key.update(0, 34))
        self.assertTrue(key.update(0, 35))
        self.assertTrue(key.update(1, 40))
        self.assertTrue(key.update(1, 64))
        self.assertFalse(key.update(1, 65))

    def test_onboard_key_uses_gpio21_pull_up(self):
        calls = {"functions": [], "pins": []}

        class FakeFPIOA:
            GPIO21 = 121

            def set_function(self, pin, function):
                calls["functions"].append((pin, function))

        class FakePin:
            IN = 1
            PULL_UP = 2

            def __init__(self, *args):
                calls["pins"].append(args)

        fake_machine = types.SimpleNamespace(FPIOA=FakeFPIOA, Pin=FakePin)
        with mock.patch.dict(sys.modules, {"machine": fake_machine}):
            key = main.init_onboard_key()

        self.assertIsInstance(key, FakePin)
        self.assertEqual(calls["functions"], [(main.ONBOARD_KEY_PIN, 121)])
        self.assertEqual(
            calls["pins"],
            [(main.ONBOARD_KEY_PIN, FakePin.IN, FakePin.PULL_UP)],
        )


class ExtendedProtocolTests(unittest.TestCase):
    def test_target_is_in_checksummed_frame(self):
        result = {
            "x_est_cm": -3.2,
            "velocity_cm_s": 4.6,
            "confidence": 0.875,
        }
        self.assertEqual(
            main.make_uart_frame(7, 1234, result, 5.0),
            "$B,7,1234,1,-32,46,875,50*7D\r\n",
        )
        self.assertEqual(
            main.make_uart_frame(8, 1300, None, -5.0),
            "$B,8,1300,0,0,0,0,-50*7C\r\n",
        )


if __name__ == "__main__":
    unittest.main()

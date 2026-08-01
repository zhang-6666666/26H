"""Desktop regression tests for the CanMV vision port."""

import unittest
from unittest import mock

import cv2
import numpy as np

import main


class DetectorTests(unittest.TestCase):
    def make_frame(self, pipe_color):
        frame = np.zeros(
            (main.FRAME_HEIGHT, main.FRAME_WIDTH, 3), dtype=np.uint8
        )
        x, y, width, height = main.ROI
        frame[y : y + height, x : x + width] = pipe_color
        return frame

    def test_green_pipe_ball(self):
        frame = self.make_frame((20, 170, 20))
        x, y, width, height = main.ROI
        center = (x + width // 2, y + height // 2)
        cv2.circle(frame, center, 10, (120, 120, 120), -1)

        detector = main.SteelBallDetector(main.ROI, main.PIPE_LENGTH_CM)
        detection, debug = detector.detect(frame, 1000)

        self.assertIsNotNone(detection)
        self.assertEqual(debug["mode"], "green")
        self.assertAlmostEqual(detection["x_raw_cm"], 0.0, delta=0.15)

    def test_green_masks_use_canmv_compatible_scalar_tuples(self):
        frame = self.make_frame((20, 170, 20))
        x, y, width, height = main.ROI
        center = (x + width // 2, y + height // 2)
        cv2.circle(frame, center, 10, (120, 120, 120), -1)
        real_in_range = main.cv2.inRange

        def strict_in_range(image, lower, upper):
            valid_types = (tuple, list, np.ndarray)
            if not isinstance(lower, valid_types) or not isinstance(
                upper, valid_types
            ):
                raise TypeError("CanMV scalar must be tuple/list/ndarray")
            return real_in_range(image, lower, upper)

        detector = main.SteelBallDetector(main.ROI, main.PIPE_LENGTH_CM)
        with mock.patch.object(main.cv2, "inRange", side_effect=strict_in_range):
            detection, _ = detector.detect(frame, 1000)
        self.assertIsNotNone(detection)

    def test_white_pipe_ball(self):
        frame = self.make_frame((235, 235, 235))
        x, y, width, height = main.ROI
        center = (x + width // 2, y + height // 2)
        cv2.circle(frame, center, 12, (45, 45, 45), -1)
        cv2.circle(
            frame,
            (center[0] - 3, center[1] - 3),
            3,
            (190, 190, 190),
            -1,
        )

        detector = main.SteelBallDetector(main.ROI, main.PIPE_LENGTH_CM)
        detection, debug = detector.detect(frame, 1000)

        self.assertIsNotNone(detection)
        self.assertEqual(debug["mode"], "white")
        self.assertAlmostEqual(detection["x_raw_cm"], 0.0, delta=0.15)

    def test_white_masks_use_canmv_compatible_scalar_tuples(self):
        frame = self.make_frame((235, 235, 235))
        x, y, width, height = main.ROI
        center = (x + width // 2, y + height // 2)
        cv2.circle(frame, center, 12, (45, 45, 45), -1)
        real_circle = main.cv2.circle

        def strict_circle(image, center_arg, radius, color, thickness):
            if not isinstance(color, (tuple, list, np.ndarray)):
                raise TypeError("CanMV scalar must be tuple/list/ndarray")
            return real_circle(image, center_arg, radius, color, thickness)

        detector = main.SteelBallDetector(main.ROI, main.PIPE_LENGTH_CM)
        with mock.patch.object(main.cv2, "circle", side_effect=strict_circle):
            detection, _ = detector.detect(frame, 1000)
        self.assertIsNotNone(detection)

    def test_empty_roi_is_invalid(self):
        frame = self.make_frame((0, 0, 0))
        detector = main.SteelBallDetector(main.ROI, main.PIPE_LENGTH_CM)
        detection, debug = detector.detect(frame, 1000)
        self.assertIsNone(detection)
        self.assertEqual(debug["mode"], "none")


class ObserverTests(unittest.TestCase):
    def test_default_hardware_link_is_uart1(self):
        self.assertEqual(main.UART_PORT, 1)
        self.assertEqual(main.UART_TX_PIN, 3)
        self.assertEqual(main.UART_RX_PIN, 4)

    def test_observer_reset_after_timeout(self):
        observer = main.AlphaBetaObserver()
        observer.update(1.0, 1000)
        observer.update(2.0, 1033)
        self.assertGreater(observer.velocity_cm_s, 0.0)

        position, velocity, acceleration = observer.update(5.0, 1300)
        self.assertEqual(position, 5.0)
        self.assertEqual(velocity, 0.0)
        self.assertEqual(acceleration, 0.0)


if __name__ == "__main__":
    unittest.main()

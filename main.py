"""01Studio CanMV K230 mini rod-ball vision program.

This is the board-side port of ``steel_ball_detector_v2.py`` from the
reference project.  It is intentionally a single file so it can be copied to
``/sdcard/main.py`` and run at power-on.

UART1 output (115200-8-N-1, ASCII):
    $B,seq,t_ms,valid,x_mm,v_mm_s,confidence,target_mm*CS\r\n

``CS`` is the two-digit XOR of the characters between ``$`` and ``*``.
``valid=0`` is sent whenever the ball is not reliable; the controller must
enter its visual-loss safe state and ignore the three zero data fields.

Requires CanMV firmware v1.7 or later with the ``cv2`` binding.
"""

import gc
import math
import os
import time

import cv2

try:
    from ulab import numpy as np
except ImportError:  # Allows recorded-video validation on a desktop.
    import numpy as np


# ---------------------------------------------------------------------------
# Installation configuration
# ---------------------------------------------------------------------------

BUILD_ID = "k230mini-ball-v1.4.0-20260801"

# The K230/K230 mini on-board camera is connected to CSI2 (id=2).
CAMERA_ID = 2
SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 960
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30

# The previous 640 x 360 ROI (0, 157, 639, 70) is scaled vertically to the
# 640 x 480 image.  Fine-tune it on the real mechanism after installation.
# Before closed-loop operation, make its left and right borders coincide with
# the two physical position references and adjust y/height around the pipe.
ROI = (0, 229, 639, 53)  # x, y, width, height
PIPE_LENGTH_CM = 25.0

# "green", "white", or "auto".  Select a fixed mode after commissioning for
# lower CPU use and less mode jitter.
PIPE_MODE = "auto"

# If the reported coordinate is reversed relative to the mechanism convention,
# change only this flag.  Positive position is normally toward image right.
POSITION_REVERSED = False

# 01Studio 2.4-inch MIPI display (640 x 480).  IDE mirroring and full-rate LCD
# refresh are unnecessary during closed-loop operation and consume bandwidth.
ENABLE_DISPLAY = True
ENABLE_IDE_MIRROR = False
DISPLAY_EVERY_N_FRAMES = 2

# K230 mini on-board programmable KEY: GPIO21, active low.
ONBOARD_KEY_PIN = 21
KEY_DEBOUNCE_MS = 25

# K230 UART1: IO3=TX1, IO4=RX1.  Only TX1 and GND are required when the
# STM32 does not send commands back to the K230.
ENABLE_UART = True
UART_PORT = 1
UART_TX_PIN = 3
UART_RX_PIN = 4
UART_BAUD = 115200

H_MIRROR = False
V_FLIP = False


# ---------------------------------------------------------------------------
# Detection and observer tuning
# ---------------------------------------------------------------------------

MAX_BALL_SPEED_CM_S = 60.0
TRACK_FORGET_FRAMES = 8
MIN_CANDIDATE_SCORE = 0.46
GREEN_FRACTION_THRESHOLD = 0.20
MIN_WHITE_ROI_FRACTION = 0.25
MIN_WHITE_ROW_FRACTION = 0.55
DARK_GRAY_MAX = 165

# Hough is used only to bridge a broken contour while a recent track exists.
# If a firmware build raises from HoughCircles, it is disabled automatically
# for the remainder of that run.
ENABLE_HOUGH_FALLBACK = True

# Low-delay alpha-beta observer used for the position/velocity UART output.
OBSERVER_ALPHA = 0.55
OBSERVER_BETA = 0.12
OBSERVER_ACCEL_NEW_WEIGHT = 0.20
OBSERVER_RESET_MS = 200

# Key task: press the on-board KEY, go to +5 cm, then command -5 cm and
# keep holding it.  A single valid frame at/over the threshold triggers the
# reversal; no acceleration estimate is used.
TASK_POSITIVE_CM = 5.0
TASK_NEGATIVE_CM = -5.0
TASK_ARRIVAL_TOLERANCE_CM = 0.3


_HSV_GREEN_LO = np.array([30, 55, 35], dtype=np.uint8)
_HSV_GREEN_HI = np.array([100, 255, 255], dtype=np.uint8)
_HSV_WHITE_LO = np.array([0, 0, 145], dtype=np.uint8)
_HSV_WHITE_HI = np.array([179, 80, 255], dtype=np.uint8)


def _clamp(value, low, high):
    return low if value < low else high if value > high else value


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(now_ms, before_ms):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now_ms, before_ms)
    return now_ms - before_ms


def _sleep_ms(delay_ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(delay_ms)
    else:
        time.sleep(delay_ms / 1000.0)


def _safe_log_ratio(value):
    return math.log(max(value, 0.001))


def _mask_border(mask, top_ratio, bottom_ratio, left_ratio, right_ratio):
    """Clear mask pixels outside the useful trough."""
    height, width = mask.shape[0], mask.shape[1]
    top = int(round(top_ratio * height))
    bottom = int(round(bottom_ratio * height))
    left = int(round(left_ratio * width))
    right = int(round(right_ratio * width))
    if top > 0:
        mask[0:top, :] = 0
    if bottom < height:
        mask[bottom:height, :] = 0
    if left > 0:
        mask[:, 0:left] = 0
    if right < width:
        mask[:, right:width] = 0


def _white_pipe_crosses_roi(white_mask):
    """Reject a white-mode ROI after the horizontal pipe leaves the view.

    The desktop reference uses the maximum white-pixel fraction of every row.
    Sampling alternate rows avoids unsupported ulab boolean reductions while
    preserving the same safety check on a pipe that occupies many ROI rows.
    """
    height, width = white_mask.shape[0], white_mask.shape[1]
    required = int(round(MIN_WHITE_ROW_FRACTION * width))
    for row in range(0, height, 2):
        if cv2.countNonZero(white_mask[row : row + 1, :]) >= required:
            return True
    return False


def _xor_checksum(payload):
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    return checksum


def make_uart_frame(seq, timestamp_ms, result, target_cm=0.0):
    """Build one checksummed measurement frame."""
    if result is None:
        valid = 0
        x_mm = 0
        velocity_mm_s = 0
        confidence = 0
    else:
        valid = 1
        x_mm = int(round(result["x_est_cm"] * 10.0))
        velocity_mm_s = int(round(result["velocity_cm_s"] * 10.0))
        confidence = int(round(_clamp(result["confidence"], 0.0, 1.0) * 1000.0))
        x_mm = int(_clamp(x_mm, -32768, 32767))
        velocity_mm_s = int(_clamp(velocity_mm_s, -32768, 32767))

    target_mm = int(round(_clamp(target_cm * 10.0, -125.0, 125.0)))
    payload = "B,%u,%u,%u,%d,%d,%u,%d" % (
        seq & 0xFFFF,
        timestamp_ms & 0xFFFFFFFF,
        valid,
        x_mm,
        velocity_mm_s,
        confidence,
        target_mm,
    )
    return "$%s*%02X\r\n" % (payload, _xor_checksum(payload))


class ActiveLowKeyDebouncer:
    """Non-blocking debounce filter for the on-board active-low key."""

    def __init__(self, debounce_ms=KEY_DEBOUNCE_MS):
        self.debounce_ms = debounce_ms
        self._raw_pressed = False
        self._stable_pressed = False
        self._raw_changed_ms = 0

    def update(self, pin_level, now_ms):
        raw_pressed = pin_level == 0
        if raw_pressed != self._raw_pressed:
            self._raw_pressed = raw_pressed
            self._raw_changed_ms = now_ms

        if (
            self._raw_pressed != self._stable_pressed
            and _ticks_diff(now_ms, self._raw_changed_ms) >= self.debounce_ms
        ):
            self._stable_pressed = self._raw_pressed

        return self._stable_pressed


class BallMission:
    """On-board-key-triggered +5 cm -> -5 cm task state machine."""

    IDLE = 0
    GO_POSITIVE = 1
    GO_NEGATIVE = 2
    HOLD_NEGATIVE = 3
    _NAMES = ("IDLE", "GO +5", "RETURN -5", "HOLD -5")

    def __init__(self):
        self.state = self.IDLE
        self.target_cm = 0.0
        self._button_down = False

    def handle_button(self, button_down):
        """Start/restart only on the button's released-to-pressed edge."""
        pressed = bool(button_down) and not self._button_down
        self._button_down = bool(button_down)
        if pressed:
            self.state = self.GO_POSITIVE
            self.target_cm = TASK_POSITIVE_CM
        return pressed

    def update_detection(self, result):
        """Advance the task from a valid estimated ball position."""
        if result is None:
            return False

        x_cm = result["x_est_cm"]
        if (
            self.state == self.GO_POSITIVE
            and x_cm >= TASK_POSITIVE_CM - TASK_ARRIVAL_TOLERANCE_CM
        ):
            self.state = self.GO_NEGATIVE
            self.target_cm = TASK_NEGATIVE_CM
            return True

        if (
            self.state == self.GO_NEGATIVE
            and x_cm <= TASK_NEGATIVE_CM + TASK_ARRIVAL_TOLERANCE_CM
        ):
            self.state = self.HOLD_NEGATIVE
            return True
        return False

    def state_name(self):
        return self._NAMES[self.state]


class AlphaBetaObserver:
    """Low-delay position and velocity observer for irregular frame timing."""

    def __init__(self):
        self.position_cm = 0.0
        self.velocity_cm_s = 0.0
        self.acceleration_cm_s2 = 0.0
        self.last_time_ms = None

    def reset(self, position_cm, now_ms):
        self.position_cm = position_cm
        self.velocity_cm_s = 0.0
        self.acceleration_cm_s2 = 0.0
        self.last_time_ms = now_ms

    def update(self, measured_cm, now_ms):
        if self.last_time_ms is None:
            self.reset(measured_cm, now_ms)
            return self.position_cm, self.velocity_cm_s, self.acceleration_cm_s2

        elapsed_ms = _ticks_diff(now_ms, self.last_time_ms)
        if elapsed_ms <= 0 or elapsed_ms > OBSERVER_RESET_MS:
            self.reset(measured_cm, now_ms)
            return self.position_cm, self.velocity_cm_s, self.acceleration_cm_s2

        dt = elapsed_ms / 1000.0
        predicted_cm = self.position_cm + self.velocity_cm_s * dt
        residual_cm = measured_cm - predicted_cm

        old_velocity = self.velocity_cm_s
        self.position_cm = predicted_cm + OBSERVER_ALPHA * residual_cm
        self.velocity_cm_s += OBSERVER_BETA * residual_cm / dt
        self.velocity_cm_s = _clamp(
            self.velocity_cm_s, -MAX_BALL_SPEED_CM_S, MAX_BALL_SPEED_CM_S
        )

        raw_acceleration = (self.velocity_cm_s - old_velocity) / dt
        self.acceleration_cm_s2 = (
            OBSERVER_ACCEL_NEW_WEIGHT * raw_acceleration
            + (1.0 - OBSERVER_ACCEL_NEW_WEIGHT) * self.acceleration_cm_s2
        )
        self.last_time_ms = now_ms
        return self.position_cm, self.velocity_cm_s, self.acceleration_cm_s2


class SteelBallDetector:
    """Fixed-ROI mirror-ball detector ported to CanMV cv2 + ulab."""

    def __init__(self, roi, pipe_length_cm):
        self.roi = roi
        self.pipe_length_cm = pipe_length_cm
        self.last_x_norm = None
        self.last_time_ms = None
        self.missed_frames = TRACK_FORGET_FRAMES
        self.hough_enabled = ENABLE_HOUGH_FALLBACK
        self.hough_error_reported = False

    def _miss(self):
        self.missed_frames = min(255, self.missed_frames + 1)

    def _temporal_score(self, cx, width, expected_x):
        if expected_x is None:
            return 1.0
        return math.exp(-5.0 * abs(cx / float(width) - expected_x))

    def _green_candidates(self, pipe_bgr, hsv, expected_x):
        height, width = pipe_bgr.shape[0], pipe_bgr.shape[1]
        expected_diameter = 0.032 * width

        # Desktop reference condition:
        #     saturation < 120 and green - max(blue, red) < 45
        # Saturating cv2.subtract plus OR implements the signed comparison
        # without int16 arrays or unsupported ulab boolean indexing.
        blue, green, red = cv2.split(pipe_bgr)
        low_saturation = cv2.inRange(hsv[:, :, 1], (0,), (119,))
        green_minus_blue = cv2.subtract(green, blue)
        green_minus_red = cv2.subtract(green, red)
        low_green_over_blue = cv2.inRange(green_minus_blue, (0,), (44,))
        low_green_over_red = cv2.inRange(green_minus_red, (0,), (44,))
        low_green_advantage = cv2.bitwise_or(
            low_green_over_blue, low_green_over_red
        )
        mask = cv2.bitwise_and(low_saturation, low_green_advantage)
        _mask_border(mask, 0.18, 0.82, 0.025, 0.975)

        kernel_size = max(5, int(round(expected_diameter * 0.15)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, width_box, height_box = cv2.boundingRect(contour)
            diameter = max(width_box, height_box)
            aspect = width_box / float(max(height_box, 1))
            vertical_box = (y + 0.5 * height_box) / float(height)
            if not (
                0.20 * expected_diameter * expected_diameter
                <= area
                <= 1.50 * expected_diameter * expected_diameter
                and 0.55 * expected_diameter <= diameter <= 1.55 * expected_diameter
                and 0.55 <= aspect <= 1.65
                and 0.18 <= vertical_box <= 0.82
            ):
                continue

            perimeter = float(cv2.arcLength(contour, True))
            circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
            center, radius = cv2.minEnclosingCircle(contour)
            cx, cy = float(center[0]), float(center[1])

            aspect_score = math.exp(-2.8 * abs(_safe_log_ratio(aspect)))
            size_score = math.exp(
                -2.5 * abs(_safe_log_ratio(diameter / expected_diameter))
            )
            circle_score = _clamp(circularity / 0.80, 0.0, 1.0)
            vertical_score = math.exp(-3.0 * abs(cy / float(height) - 0.50))
            temporal_score = self._temporal_score(cx, width, expected_x)
            score = (
                0.28 * circle_score
                + 0.23 * aspect_score
                + 0.20 * size_score
                + 0.17 * vertical_score
                + 0.12 * temporal_score
            )
            candidates.append((score, cx, cy, float(radius), "green"))
        return candidates

    def _white_candidates(self, pipe_bgr, hsv, expected_x):
        height, width = pipe_bgr.shape[0], pipe_bgr.shape[1]
        expected_diameter = 0.040 * width
        gray = cv2.cvtColor(pipe_bgr, cv2.COLOR_BGR2GRAY)
        _, dark = cv2.threshold(gray, DARK_GRAY_MAX, 255, cv2.THRESH_BINARY_INV)
        _mask_border(dark, 0.10, 0.90, 0.10, 0.96)

        close_size = max(3, int(round(0.22 * expected_diameter)))
        if close_size % 2 == 0:
            close_size += 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_size, close_size)
        )
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)
        white = cv2.inRange(hsv, _HSV_WHITE_LO, _HSV_WHITE_HI)
        contours, _ = cv2.findContours(
            dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, width_box, height_box = cv2.boundingRect(contour)
            diameter = max(width_box, height_box)
            aspect = width_box / float(max(height_box, 1))
            if not (
                0.14 * expected_diameter * expected_diameter
                <= area
                <= 1.60 * expected_diameter * expected_diameter
                and 0.55 * expected_diameter <= diameter <= 1.65 * expected_diameter
                and 0.55 <= aspect <= 1.60
            ):
                continue

            perimeter = float(cv2.arcLength(contour, True))
            circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
            center, radius = cv2.minEnclosingCircle(contour)
            cx, cy = float(center[0]), float(center[1])

            # Verify that the dark core is surrounded by bright, low-saturation
            # pipe.  Small local masks replace desktop np.ogrid/boolean indexing.
            reach = max(4, int(round(2.35 * radius)))
            x0, y0 = max(0, int(cx) - reach), max(0, int(cy) - reach)
            x1 = min(width, int(cx) + reach + 1)
            y1 = min(height, int(cy) + reach + 1)
            sub_width, sub_height = x1 - x0, y1 - y0
            if sub_width < 5 or sub_height < 5:
                continue

            local_center = (int(round(cx)) - x0, int(round(cy)) - y0)
            inner_mask = np.zeros((sub_height, sub_width), dtype=np.uint8)
            ring_mask = np.zeros((sub_height, sub_width), dtype=np.uint8)
            cv2.circle(
                inner_mask,
                local_center,
                max(2, int(round(0.80 * radius))),
                (255,),
                -1,
            )
            cv2.circle(
                ring_mask,
                local_center,
                max(3, int(round(2.25 * radius))),
                (255,),
                -1,
            )
            cv2.circle(
                ring_mask,
                local_center,
                max(2, int(round(1.30 * radius))),
                (0,),
                -1,
            )

            inner_count = cv2.countNonZero(inner_mask)
            ring_count = cv2.countNonZero(ring_mask)
            if inner_count < 8 or ring_count < 20:
                continue

            gray_sub = gray[y0:y1, x0:x1]
            white_sub = white[y0:y1, x0:x1]
            white_in_ring = cv2.bitwise_and(white_sub, ring_mask)
            white_fraction = cv2.countNonZero(white_in_ring) / float(ring_count)
            inner_mean = float(cv2.mean(gray_sub, inner_mask)[0])
            ring_mean = float(cv2.mean(gray_sub, ring_mask)[0])
            contrast = ring_mean - inner_mean

            _, dark_inner = cv2.threshold(
                gray_sub, 185, 255, cv2.THRESH_BINARY_INV
            )
            dark_inner = cv2.bitwise_and(dark_inner, inner_mask)
            dark_fraction = cv2.countNonZero(dark_inner) / float(inner_count)
            if white_fraction < 0.42 or dark_fraction < 0.22:
                continue

            aspect_score = math.exp(-2.8 * abs(_safe_log_ratio(aspect)))
            size_score = math.exp(
                -2.5 * abs(_safe_log_ratio(diameter / expected_diameter))
            )
            circle_score = _clamp(circularity / 0.75, 0.0, 1.0)
            white_score = _clamp(white_fraction / 0.85, 0.0, 1.0)
            contrast_score = _clamp(contrast / 80.0, 0.0, 1.0)
            temporal_score = self._temporal_score(cx, width, expected_x)
            score = (
                0.23 * circle_score
                + 0.20 * aspect_score
                + 0.19 * size_score
                + 0.20 * white_score
                + 0.12 * contrast_score
                + 0.06 * temporal_score
            )
            candidates.append((score, cx, cy, float(radius), "white"))
        return candidates

    def _hough_fallback(self, pipe_bgr, expected_x):
        if expected_x is None or not self.hough_enabled:
            return None

        height, width = pipe_bgr.shape[0], pipe_bgr.shape[1]
        expected_radius = 0.016 * width
        gray = cv2.cvtColor(pipe_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 1.5)
        try:
            circles = cv2.HoughCircles(
                gray,
                3,  # CanMV documents 3 as HOUGH_GRADIENT.
                1.2,
                max(12, height // 2),
                param1=100,
                param2=18,
                minRadius=max(4, int(round(0.55 * expected_radius))),
                maxRadius=max(7, int(round(1.55 * expected_radius))),
            )
        except Exception as error:
            self.hough_enabled = False
            if not self.hough_error_reported:
                print("Hough disabled after cv2 error: %s" % error)
                self.hough_error_reported = True
            return None

        if circles is None:
            return None

        # CanMV returns Nx3; desktop OpenCV commonly returns 1xNx3.
        shape = circles.shape
        count = shape[1] if len(shape) == 3 else shape[0]
        best = None
        for index in range(count):
            circle = circles[0, index] if len(shape) == 3 else circles[index]
            cx, cy, radius = float(circle[0]), float(circle[1]), float(circle[2])
            vertical_score = math.exp(-3.0 * abs(cy / float(height) - 0.50))
            size_score = math.exp(
                -2.5 * abs(_safe_log_ratio(radius / expected_radius))
            )
            temporal_score = self._temporal_score(cx, width, expected_x)
            score = (
                0.45 * vertical_score
                + 0.35 * size_score
                + 0.20 * temporal_score
            )
            if best is None or score > best[0]:
                best = (score, cx, cy, radius, "hough")
        if best is None or best[0] < 0.50:
            return None
        return best

    def _gate_norm(self, now_ms):
        if self.last_time_ms is None:
            return 1.0
        dt = max(0.001, _ticks_diff(now_ms, self.last_time_ms) / 1000.0)
        physical_step = MAX_BALL_SPEED_CM_S * dt / self.pipe_length_cm
        return min(0.22, 0.025 + physical_step + 0.02 * min(self.missed_frames, 3))

    def detect(self, frame_bgr, now_ms):
        rx, ry, rw, rh = self.roi
        frame_height, frame_width = frame_bgr.shape[0], frame_bgr.shape[1]
        if (
            rx < 0
            or ry < 0
            or rw <= 0
            or rh <= 0
            or rx + rw > frame_width
            or ry + rh > frame_height
        ):
            raise ValueError("ROI exceeds frame; check FRAME_WIDTH/HEIGHT and ROI")

        pipe = frame_bgr[ry : ry + rh, rx : rx + rw]
        hsv = cv2.cvtColor(pipe, cv2.COLOR_BGR2HSV)
        expected_x = (
            self.last_x_norm
            if self.missed_frames < TRACK_FORGET_FRAMES
            else None
        )

        green_mask = cv2.inRange(hsv, _HSV_GREEN_LO, _HSV_GREEN_HI)
        green_fraction = cv2.countNonZero(green_mask) / float(rw * rh)
        white_mask = cv2.inRange(hsv, _HSV_WHITE_LO, _HSV_WHITE_HI)
        white_fraction = cv2.countNonZero(white_mask) / float(rw * rh)

        mode = PIPE_MODE
        if mode == "auto":
            mode = "green" if green_fraction >= GREEN_FRACTION_THRESHOLD else "white"

        if mode == "green":
            candidates = self._green_candidates(pipe, hsv, expected_x)
        elif mode == "white":
            if (
                white_fraction < MIN_WHITE_ROI_FRACTION
                or not _white_pipe_crosses_roi(white_mask)
            ):
                self._miss()
                return None, {
                    "mode": "none",
                    "candidates": 0,
                    "green_fraction": green_fraction,
                    "white_fraction": white_fraction,
                }
            candidates = self._white_candidates(pipe, hsv, expected_x)
        else:
            raise ValueError("PIPE_MODE must be 'green', 'white', or 'auto'")

        debug = {
            "mode": mode,
            "candidates": len(candidates),
            "green_fraction": green_fraction,
            "white_fraction": white_fraction,
        }

        gate = self._gate_norm(now_ms)
        plausible = []
        for item in candidates:
            image_x_norm = item[1] / float(max(rw - 1, 1))
            if expected_x is None or abs(image_x_norm - expected_x) <= gate:
                plausible.append(item)

        best = max(plausible, key=lambda item: item[0]) if plausible else None
        if best is None or best[0] < MIN_CANDIDATE_SCORE:
            best = self._hough_fallback(pipe, expected_x)
            if best is None:
                self._miss()
                return None, debug
            hough_norm = best[1] / float(max(rw - 1, 1))
            if expected_x is not None and abs(hough_norm - expected_x) > gate:
                self._miss()
                return None, debug

        score, cx_local, cy_local, radius, source = best
        image_x_norm = _clamp(cx_local / float(max(rw - 1, 1)), 0.0, 1.0)
        position_norm = 1.0 - image_x_norm if POSITION_REVERSED else image_x_norm
        x_raw_cm = (position_norm - 0.5) * self.pipe_length_cm

        self.last_x_norm = image_x_norm
        self.last_time_ms = now_ms
        self.missed_frames = 0
        return {
            "cx": rx + cx_local,
            "cy": ry + cy_local,
            "radius": radius,
            "confidence": _clamp(score, 0.0, 0.99),
            "source": source,
            "image_x_norm": image_x_norm,
            "x_raw_cm": x_raw_cm,
            # Reset the observer on first acquisition or after the detector has
            # intentionally forgotten an old track.
            "track_reset": expected_x is None,
        }, debug


def annotate(frame, detector, result, debug, fps, mission):
    rx, ry, rw, rh = detector.roi
    cv2.rectangle(frame, (rx, ry), (rx + rw - 1, ry + rh - 1), (255, 120, 0), 2)

    # Marks make ROI/coordinate calibration directly visible in the IDE.
    for x_cm, color in (
        (-5.0, (255, 255, 0)),
        (0.0, (0, 255, 0)),
        (5.0, (255, 255, 0)),
    ):
        position_norm = x_cm / detector.pipe_length_cm + 0.5
        image_norm = 1.0 - position_norm if POSITION_REVERSED else position_norm
        px = rx + int(round(image_norm * (rw - 1)))
        cv2.line(frame, (px, ry), (px, ry + rh - 1), color, 1)

    if result is None:
        label = "BALL LOST  fps=%.1f" % fps
        color = (0, 0, 255)
    else:
        cx, cy = int(round(result["cx"])), int(round(result["cy"]))
        cv2.circle(frame, (cx, cy), int(round(result["radius"])), (0, 0, 255), 2)
        cv2.drawMarker(frame, (cx, cy), (255, 0, 255), cv2.MARKER_CROSS, 16, 1)
        label = "x=%+.2fcm v=%+.1fcm/s c=%.2f fps=%.1f" % (
            result["x_est_cm"],
            result["velocity_cm_s"],
            result["confidence"],
            fps,
        )
        color = (0, 255, 0)

    cv2.putText(
        frame,
        label,
        (12, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )
    task_label = "%s  target=%+.1fcm" % (
        mission.state_name(),
        mission.target_cm,
    )
    cv2.putText(
        frame,
        task_label,
        (12, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )

    key_text = "KEY21: START" if mission.state == mission.IDLE else "KEY21: RESTART"
    cv2.putText(
        frame,
        key_text,
        (500, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )
    debug_label = "mode=%s cand=%d G=%.0f%% W=%.0f%% miss=%d" % (
        debug["mode"],
        debug["candidates"],
        debug["green_fraction"] * 100.0,
        debug["white_fraction"] * 100.0,
        detector.missed_frames,
    )
    cv2.putText(
        frame,
        debug_label,
        (12, FRAME_HEIGHT - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )


def init_uart():
    if not ENABLE_UART:
        return None

    from machine import FPIOA, UART

    fpioa = FPIOA()
    if UART_PORT == 1:
        fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
        uart_id = UART.UART1
    elif UART_PORT == 2:
        fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
        uart_id = UART.UART2
    else:
        raise ValueError("UART_PORT must be 1 or 2")

    return UART(
        uart_id,
        baudrate=UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def init_onboard_key():
    from machine import FPIOA, Pin

    fpioa = FPIOA()
    fpioa.set_function(ONBOARD_KEY_PIN, FPIOA.GPIO21)
    return Pin(ONBOARD_KEY_PIN, Pin.IN, Pin.PULL_UP)


def main():
    from media.display import Display
    from media.media import MediaManager
    from media.sensor import Sensor

    sensor = None
    uart = None
    onboard_key = None
    display_started = False
    media_started = False
    detector = SteelBallDetector(ROI, PIPE_LENGTH_CM)
    observer = AlphaBetaObserver()
    mission = BallMission()
    key_debouncer = ActiveLowKeyDebouncer()
    seq = 0

    try:
        sensor = Sensor(
            id=CAMERA_ID,
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=CAMERA_FPS,
        )
        sensor.reset()
        sensor.set_hmirror(H_MIRROR)
        sensor.set_vflip(V_FLIP)
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)  # Required by CanMV cv2.

        if ENABLE_DISPLAY:
            Display.init(
                Display.ST7701,
                width=FRAME_WIDTH,
                height=FRAME_HEIGHT,
                to_ide=ENABLE_IDE_MIRROR,
            )
            display_started = True

        MediaManager.init()
        media_started = True
        uart = init_uart()
        onboard_key = init_onboard_key()
        sensor.run()

        fps_value = 0.0
        fps_frames = 0
        fps_start_ms = _ticks_ms()
        print("K230 mini steel-ball detector: %s" % BUILD_ID)
        print(
            "camera=CSI%d frame=%dx%d roi=%s pipe=%s uart=%s key=GPIO%d"
            % (
                CAMERA_ID,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                str(ROI),
                PIPE_MODE,
                "UART%d" % UART_PORT if ENABLE_UART else "off",
                ONBOARD_KEY_PIN,
            )
        )

        while True:
            os.exitpoint()
            image = sensor.snapshot()
            frame = image.to_numpy_ref()
            now_ms = _ticks_ms()

            if onboard_key is not None:
                key_pressed = key_debouncer.update(onboard_key.value(), now_ms)
                if mission.handle_button(key_pressed):
                    print("task start: target=+%.1fcm" % TASK_POSITIVE_CM)

            fps_frames += 1
            fps_elapsed_ms = _ticks_diff(now_ms, fps_start_ms)
            if fps_elapsed_ms >= 500:
                fps_value = fps_frames * 1000.0 / fps_elapsed_ms
                fps_frames = 0
                fps_start_ms = now_ms

            detection, debug = detector.detect(frame, now_ms)
            result = None
            if detection is not None:
                if detection["track_reset"]:
                    observer.reset(detection["x_raw_cm"], now_ms)
                    x_est = observer.position_cm
                    velocity = observer.velocity_cm_s
                    acceleration = observer.acceleration_cm_s2
                else:
                    x_est, velocity, acceleration = observer.update(
                        detection["x_raw_cm"], now_ms
                    )
                detection["x_est_cm"] = x_est
                detection["velocity_cm_s"] = velocity
                detection["acceleration_cm_s2"] = acceleration
                result = detection

            if mission.update_detection(result):
                print(
                    "task state=%s target=%+.1fcm"
                    % (mission.state_name(), mission.target_cm)
                )

            if uart is not None:
                uart.write(
                    make_uart_frame(seq, now_ms, result, mission.target_cm)
                )

            if ENABLE_DISPLAY and seq % DISPLAY_EVERY_N_FRAMES == 0:
                annotate(frame, detector, result, debug, fps_value, mission)
                Display.show_image(image, x=0, y=0)

            if seq % 15 == 0:
                if result is None:
                    print("seq=%u LOST fps=%.1f" % (seq, fps_value))
                else:
                    print(
                        "seq=%u x=%+.3fcm v=%+.2fcm/s conf=%.2f src=%s fps=%.1f"
                        % (
                            seq,
                            result["x_est_cm"],
                            result["velocity_cm_s"],
                            result["confidence"],
                            result["source"],
                            fps_value,
                        )
                    )

            seq = (seq + 1) & 0xFFFF
            if seq % 10 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stopped")
    except BaseException as error:
        print("fatal error: %s" % error)
        raise
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
        if display_started:
            try:
                Display.deinit()
            except Exception:
                pass
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass
        _sleep_ms(100)
        if media_started:
            try:
                MediaManager.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    main()

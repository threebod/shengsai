"""Standalone MaixCAM2 stop prediction experiment: prints only, no UART."""
import math
import time

COLORS = (5, 4, 2)  # Current tray: black, green, yellow.
# Snapshot of vision_settings.py; kept local for MaixVision single-file runs.
COLOR_THRESHOLDS = {
    1: [21, 50, 32, 68, 15, 50],
    2: [50, 105, -30, 25, 35, 100],
    3: [9, 46, -6, 30, -60, -10],
    4: [37, 70, -70, -38, 28, 65],
    5: [8, 25, -14, 10, -8, 16],
    6: [40, 75, -29, -5, -33, -8],
}
COLOR_NAMES = {1: "red", 2: "yellow", 3: "blue", 4: "green",
               5: "black", 6: "light_blue"}
LEAD_MS = 350  # Experimental early-warning horizon, not an arm motion duration.
MIN_INTERVAL_MS = 60
MAX_GAP_MS = 250
MAX_STEP_PX = 50
MIN_SPEED = 8  # All motion units use 320x240 coordinates: px/s and px/s^2.
MIN_DECEL = 15
LOG_INTERVAL_MS = 200
TRACE_MOTION = True  # One raw-coordinate line per frame for offline replay.


def find_materials(img):
    # Match production tray detection at 640x480: largest blob per color,
    # original area limits and suppression of overlapping color boxes.
    candidates = []
    for color in sorted(COLORS):
        blobs = img.find_blobs([COLOR_THRESHOLDS[color]], roi=(0, 0, 640, 480),
                               area_threshold=50, pixels_threshold=50)
        if not blobs:
            continue
        blob = max(blobs, key=lambda blob: blob.area())
        area = blob.area()
        if (4000 if color == 5 else 1500) <= area <= 30000:
            candidates.append((area, color, blob))
    candidates.sort(key=lambda candidate: candidate[0], reverse=True)
    kept, points = [], {}
    for area, color, blob in candidates:
        x, y, width, height = blob.x(), blob.y(), blob.w(), blob.h()
        if any(min(x + width, bx + bw) > max(x, bx) and
               min(y + height, by + bh) > max(y, by) for bx, by, bw, bh in kept):
            continue
        kept.append((x, y, width, height))
        points[color] = (int(round(blob.cx() / 2)), int(round(blob.cy() / 2)),
                         min(100, int(blob.pixels() * 100 / max(area, 1))),
                         int(round(width / 2)), int(round(height / 2)), color)
    return points


def color_region(point):
    if point[1] >= 120:
        return "bottom"
    if point[0] == 160:
        return "upper_boundary"
    return "upper_left" if point[0] < 160 else "upper_right"


class StopPredictor:
    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = []
        self.still_samples = []
        self.deadline_ms = None

    def update(self, point, now_ms):
        result = {"state": "collecting", "eta_ms": None, "speed": None,
                  "decel": None, "rmse": None, "still_10f": False,
                  "reason": "samples", "window_ms": 0}
        if point is None:
            self.reset()
            result["state"] = "lost"
            result["reason"] = "missing"
            return result
        too_fast = False
        if self.samples:
            previous_ms, previous = self.samples[-1]
            gap = now_ms - previous_ms
            if (gap <= 0 or gap > MAX_GAP_MS or point[5] != previous[5] or
                    math.hypot(point[0] - previous[0], point[1] - previous[1]) > MAX_STEP_PX):
                self.reset()
            elif gap < MIN_INTERVAL_MS:
                too_fast = True
        # The ten-frame comparison uses every frame, independently of speed sampling.
        self.still_samples = (self.still_samples + [point])[-10:]
        if not too_fast:
            self.samples = (self.samples + [(now_ms, point)])[-9:]
        result["still_10f"] = len(self.still_samples) == 10 and all(
                max(row[axis] for row in self.still_samples) -
                min(row[axis] for row in self.still_samples) <= 8 and
                abs(sorted(row[axis] for row in self.still_samples[:5])[2] -
                    sorted(row[axis] for row in self.still_samples[-5:])[2]) <= 2
                for axis in (0, 1))
        if too_fast:
            # Ignore oversampled frames without repeating an older forecast.
            return None
        # Two-interval chords reduce the effect of integer-center jitter.
        window = self.samples[-9:]
        result["window_ms"] = window[-1][0] - window[0][0]
        if len(window) < 7 or window[-1][0] - window[0][0] < 350:
            self.deadline_ms = None
            return result
        times, speeds = [], []
        for (t0, p0), (t1, p1) in zip(window, window[2:]):
            times.append(((t0 + t1) / 2 - now_ms) / 1000)
            speeds.append(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) * 1000 / (t1 - t0))
        mean_t, mean_v = sum(times) / len(times), sum(speeds) / len(speeds)
        slope = sum((t - mean_t) * (v - mean_v) for t, v in zip(times, speeds)) / sum(
            (t - mean_t) ** 2 for t in times)
        speed = mean_v - slope * mean_t  # Fitted speed at this sample's timestamp.
        rmse = math.sqrt(sum((v - speed - slope * t) ** 2
                             for t, v in zip(times, speeds)) / len(times))
        result.update(state="moving", speed=max(0, speed), decel=-slope, rmse=rmse)
        decreasing = speeds[0] - speeds[-1] >= max(6, speeds[0] * 0.2)
        if speed < MIN_SPEED:
            result["reason"] = "below_speed"
        elif slope > -MIN_DECEL:
            result["reason"] = "decel_small"
        elif not decreasing:
            result["reason"] = "drop_small"
        elif rmse > max(4, mean_v * 0.15):
            result["reason"] = "fit_residual"
        else:
            result["reason"] = "fit_ok"
        if result["reason"] != "fit_ok":
            self.deadline_ms = None
            return result
        eta_ms = speed / -slope * 1000
        result.update(state="decelerating", eta_ms=eta_ms, reason="outside_lead")
        deadline = now_ms + eta_ms
        if eta_ms <= LEAD_MS and color_region(point) != "upper_boundary":
            result["reason"] = "await_confirmation"
            # Require two adjacent estimates of the stop time to agree.
            if self.deadline_ms is not None and abs(deadline - self.deadline_ms) <= 100:
                result["state"] = "early_hint"
                result["reason"] = "consistent"
            elif self.deadline_ms is not None:
                result["reason"] = "deadline_changed"
            self.deadline_ms = deadline
        else:
            self.deadline_ms = None
            if color_region(point) == "upper_boundary":
                result["reason"] = "region_boundary"
        return result


def main():
    from maix import app, camera, display, err, gpio, image, pinmap

    if len(COLORS) != 3 or len(set(COLORS)) != 3 or any(c not in COLOR_NAMES for c in COLORS):
        raise ValueError("COLORS must contain three distinct IDs from 1 to 6")
    predictors = {c: StopPredictor() for c in COLORS}
    last_states, last_still, last_logs = {}, {}, {}
    err.check_raise(pinmap.set_pin_function("B25", "GPIOB25"), "illumination GPIO mapping")
    lamp = gpio.GPIO("GPIOB25", gpio.Mode.OUT)
    lamp.value(0)
    cam = None
    print("STOP TEST colors={} lead_ms={} lamp=OFF robot_commands=OFF trace={}".format(
        COLORS, LEAD_MS, int(TRACE_MOTION)))
    print("early_hint is a forecast, not measured stillness; regions are CURRENT image regions")
    print("Centers measure motion only; no alignment/anchor-distance check")
    try:
        cam = camera.Camera(640, 480)
        screen = display.Display()
        warmup_until = int(time.monotonic() * 1000) + 2000
        while not app.need_exit():
            captured_ms = int(time.monotonic() * 1000)
            frame = cam.read()
            if captured_ms < warmup_until:
                continue
            points = find_materials(frame) if frame is not None else {}
            for line, color in enumerate(COLORS):
                point = points.get(color)
                result = predictors[color].update(point, captured_ms)
                if result is None:
                    continue
                state = result["state"]
                now_ms = int(time.monotonic() * 1000)
                age_ms = now_ms - captured_ms
                # Account for capture/detection delay; an expired forecast cannot be an early hint.
                remaining_ms = (None if result["eta_ms"] is None else result["eta_ms"] - age_ms)
                if state == "early_hint" and remaining_ms <= 0:
                    state = "forecast_expired"
                    result["reason"] = "processing_delay"
                region = color_region(point) if point else "unknown"
                if last_states.get(color) == "early_hint" and state != "early_hint":
                    print("CANCEL t_ms={} color={} reason={}".format(now_ms, COLOR_NAMES[color], state))
                if (state != last_states.get(color) or result["still_10f"] != last_still.get(color) or
                        now_ms - last_logs.get(color, 0) >= LOG_INTERVAL_MS):
                    number = lambda value: "NA" if value is None else "{:.1f}".format(value)
                    print("STOP t_ms={} color={} region={} state={} eta_ms={} speed={} decel={} rmse={} age_ms={} still_10f={} reason={} window_ms={}".format(
                        now_ms, COLOR_NAMES[color], region, state, number(remaining_ms),
                        number(result["speed"]), number(result["decel"]), number(result["rmse"]), age_ms,
                        int(result["still_10f"]), result["reason"], result["window_ms"]))
                    last_states[color], last_logs[color] = state, now_ms
                    last_still[color] = result["still_10f"]
                if frame is not None:
                    frame.draw_string(2, 2 + line * 24, "{} {} {} still={} eta={}ms".format(
                        COLOR_NAMES[color], region, state, int(result["still_10f"]),
                        "NA" if remaining_ms is None else int(remaining_ms)), image.COLOR_YELLOW)
                    if point:
                        frame.draw_cross(point[0] * 2, point[1] * 2, image.COLOR_GREEN, 6, 1)
            if TRACE_MOTION:
                coordinates = ["{}={}".format(COLOR_NAMES[color],
                               "{},{}".format(*points[color][:2]) if color in points else "NA")
                               for color in COLORS]
                print("SAMPLE t_ms={} age_ms={} {}".format(
                    captured_ms, int(time.monotonic() * 1000) - captured_ms, " ".join(coordinates)))
            if frame is not None:
                frame.draw_line(0, 240, 639, 240, image.COLOR_YELLOW, 1)
                frame.draw_line(320, 0, 320, 240, image.COLOR_YELLOW, 1)
                screen.show(frame)
    finally:
        lamp.value(0)
        if cam is not None:
            cam.close()


if __name__ == "__main__":
    main()

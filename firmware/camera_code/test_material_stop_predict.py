import math
import pathlib
import subprocess
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from material_stop_predict_test import StopPredictor, color_region
import material_stop_predict_test as entry


def point(x, y=190, color=4):
    return (x, y, 80, 30, 30, color)


class StopPredictorTest(unittest.TestCase):
    def test_script_imports_without_other_project_files(self):
        result = subprocess.run(
            [sys.executable, "-I", "-c",
             "import runpy, sys; runpy.run_path(sys.argv[1], run_name='standalone_test')",
             str(pathlib.Path(entry.__file__).resolve())],
            capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)

    def feed(self, trajectory, times=range(0, 1401, 80)):
        predictor = StopPredictor()
        return [(ms, predictor.update(trajectory(ms / 1000), ms)) for ms in times]

    def test_deceleration_predicts_before_stop_without_center_alignment(self):
        for origin in (30, 190):
            rows = self.feed(lambda t: point(origin + 60 * min(t, 1) - 30 * min(t, 1) ** 2))
            early = [(ms, row) for ms, row in rows if row["state"] == "early_hint"]
            self.assertTrue(early)
            for ms, row in early:
                self.assertLess(ms, 1000)
                self.assertAlmostEqual(1000 - ms, row["eta_ms"], places=5)

    def test_rotating_material_decelerates_to_stop(self):
        def trajectory(t):
            t = min(t, 1)
            angle = 0.8 * t - 0.4 * t ** 2
            return point(round(160 + 70 * math.cos(angle)), round(100 + 70 * math.sin(angle)))
        early = [(ms, row) for ms, row in self.feed(trajectory) if row["state"] == "early_hint"]
        self.assertTrue(early)
        self.assertTrue(all(ms < 1000 for ms, _ in early))

    def test_constant_speed_and_acceleration_do_not_predict_stop(self):
        for trajectory in (lambda t: point(20 + 40 * t),
                           lambda t: point(20 + 10 * t + 20 * t ** 2)):
            self.assertFalse(any(row["state"] == "early_hint" for _, row in self.feed(trajectory)))

    def test_stationary_jitter_is_not_early_prediction(self):
        rows = self.feed(lambda t: point(80 + (int(round(t / 0.08)) % 3 - 1), 60))
        self.assertFalse(any(row["state"] == "early_hint" for _, row in rows))
        self.assertTrue(rows[-1][1]["still_10f"])

    def test_deceleration_far_from_stop_has_no_early_hint(self):
        rows = self.feed(lambda t: point(20 + 50 * t - 12.5 * t ** 2), range(0, 801, 80))
        self.assertTrue(any(row["state"] == "decelerating" for _, row in rows))
        self.assertFalse(any(row["state"] == "early_hint" for _, row in rows))

    def test_irregular_timestamps_predict_real_elapsed_time(self):
        times = (0, 70, 160, 240, 310, 410, 490, 560, 650, 730, 810, 900)
        rows = self.feed(lambda t: point(30 + 60 * t - 30 * t ** 2), times)
        early = [(ms, row) for ms, row in rows if row["state"] == "early_hint"]
        self.assertTrue(early)
        for ms, row in early:
            self.assertAlmostEqual(1000 - ms, row["eta_ms"], places=5)

    def test_missing_gap_jump_color_change_and_backwards_clock_reset(self):
        cases = ((None, 600), (point(50), 1100), (point(200), 600),
                 (point(50, color=1), 600), (point(50), 400))
        for next_point, next_ms in cases:
            with self.subTest(next_point=next_point, next_ms=next_ms):
                predictor = StopPredictor()
                for ms in range(0, 561, 80):
                    predictor.update(point(30 + ms * 0.04), ms)
                result = predictor.update(next_point, next_ms)
                self.assertIn(result["state"], ("lost", "collecting"))
                self.assertIsNone(predictor.deadline_ms)
                self.assertLessEqual(len(predictor.samples), 1)

    def test_first_near_stop_estimate_is_not_enough_for_hint(self):
        rows = self.feed(lambda t: point(30 + 60 * t - 30 * t ** 2), range(0, 721, 80))
        self.assertEqual("decelerating", rows[-1][1]["state"])
        self.assertLess(rows[-1][1]["eta_ms"], 350)

    def test_fast_frames_do_not_prevent_consecutive_forecasts(self):
        predictor = StopPredictor()
        results = []
        for ms in range(0, 901, 20):
            t = ms / 1000
            row = predictor.update(point(30 + 60 * t - 30 * t ** 2), ms)
            if row is not None:
                results.append(row)
        self.assertTrue(any(row["state"] == "early_hint" for row in results))

    def test_ten_frame_comparison_does_not_interrupt_an_active_forecast(self):
        predictor = StopPredictor()
        results = []
        for ms in range(0, 841, 20):
            t = ms / 1000
            result = predictor.update(point(30 + 60 * t - 30 * t ** 2), ms)
            if result is not None:
                results.append(result)
        self.assertTrue(any(row["state"] == "early_hint" and row.get("still_10f")
                            for row in results))

    def test_low_speed_reports_fit_even_when_ten_frame_comparison_passes(self):
        rows = self.feed(lambda t: point(30 + 5 * t))
        result = rows[-1][1]
        self.assertTrue(result.get("still_10f"))
        self.assertAlmostEqual(5, result["speed"])
        self.assertEqual("below_speed", result["reason"])

    def test_still_comparison_counts_every_camera_frame(self):
        predictor = StopPredictor()
        for ms in range(0, 180, 20):
            result = predictor.update(point(50), ms)
            self.assertTrue(result is None or not result["still_10f"])
        self.assertTrue(predictor.update(point(50), 180)["still_10f"])

    def test_sudden_stop_is_confirmed_but_not_predicted(self):
        rows = self.feed(lambda t: point(20 + 40 * min(t, 0.8)), range(0, 2401, 80))
        self.assertFalse(any(ms < 800 and row["state"] == "early_hint" for ms, row in rows))
        self.assertTrue(rows[-1][1]["still_10f"])

    def test_regions_match_existing_image_partition(self):
        for p, expected in ((point(20, 120), "bottom"), (point(20, 60), "upper_left"),
                            (point(200, 60), "upper_right"), (point(160, 60), "upper_boundary")):
            self.assertEqual(expected, color_region(p))

    def test_camera_entry_prints_real_colors_regions_and_never_opens_uart(self):
        maix = types.ModuleType("maix")
        for name in ("app", "camera", "display", "err", "gpio", "image", "pinmap", "uart"):
            setattr(maix, name, Mock())
        clock = [0]
        times = iter(range(2100, 4101, 80))

        def need_exit():
            now = next(times, None)
            if now is None:
                return True
            clock[0] = now
            return False

        def detect(frame):
            t = min((clock[0] - 2100) / 1000, 1)
            return {4: point(30 + 60 * t - 30 * t ** 2)}

        maix.app.need_exit.side_effect = need_exit
        with patch.dict(sys.modules, {"maix": maix}), \
                patch.object(entry.time, "monotonic", side_effect=lambda: clock[0] / 1000), \
                patch.object(entry, "find_materials", side_effect=detect), \
                patch("builtins.print") as output:
            entry.main()
        lines = [args[0] for args, _ in output.call_args_list]
        self.assertTrue(any("color=green region=bottom state=early_hint" in line for line in lines))
        self.assertTrue(any("color=green region=bottom" in line and "still_10f=1" in line for line in lines))
        self.assertTrue(any("color=black region=unknown state=lost" in line for line in lines))
        samples = [line for line in lines if line.startswith("SAMPLE ")]
        self.assertEqual(len(range(2100, 4101, 80)), len(samples))
        self.assertIn("t_ms=2100 age_ms=0 black=NA green=30.0,190 yellow=NA", samples[0])
        self.assertTrue(any("reason=below_speed" in line for line in lines))
        maix.uart.UART.assert_not_called()
        maix.camera.Camera.return_value.close.assert_called_once()
        self.assertTrue(all(call.args == (0,) for call in maix.gpio.GPIO.return_value.value.call_args_list))

    def test_embedded_detection_matches_production_tray_candidates(self):
        from material_layout import find_material_layout

        def blob(x, y, width, height):
            item = Mock()
            for name, value in (("x", x), ("y", y), ("w", width), ("h", height),
                                ("area", width * height), ("pixels", width * height),
                                ("cx", x + width // 2), ("cy", y + height // 2)):
                getattr(item, name).return_value = value
            return item

        for colors in ((1, 4, 6), (2, 3, 5)):
            a, b, c = colors
            scenarios = ({}, {a: [blob(100, 300, 60, 60)]},
                         {a: [blob(100, 300, 60, 60)], b: [blob(300, 80, 80, 80)],
                          c: [blob(500, 80, 60, 60)]},
                         {a: [blob(100, 100, 60, 60)], b: [blob(110, 110, 80, 80)]},
                         {a: [blob(100, 100, 40, 40), blob(80, 80, 200, 200)]},
                         {c: [blob(100, 100, 50, 50)]})
            for blobs in scenarios:
                with self.subTest(colors=colors, blobs=blobs):
                    frame = Mock()
                    frame.find_blobs.side_effect = lambda thresholds, **kwargs: blobs.get(
                        next(color for color, threshold in entry.COLOR_THRESHOLDS.items()
                             if thresholds == [threshold]), [])
                    request = {"selector": 0, "values": [0, 0, 320, 240, 160, 120,
                               sum(1 << (color - 1) for color in colors), 3]}
                    expected = find_material_layout(frame, request)
                    with patch.object(entry, "COLORS", colors):
                        found = entry.find_materials(frame)
                    self.assertEqual({p[5]: p for p in expected["visible_points"]} if expected else {}, found)


if __name__ == "__main__":
    unittest.main()

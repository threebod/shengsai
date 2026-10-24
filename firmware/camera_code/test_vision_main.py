import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_protocol import (FLAG_RING_NO_DIGIT, FLAG_STACK_PREPARE, FLAG_STABLE,
                             FLAG_TRACKING, FLAG_VALID, KIND_REQUEST,
                             MODE_MATERIAL, MODE_RING, Parser, encode)


class VisionMainTest(unittest.TestCase):
    def run_camera(self, incoming, times, confirmations=None, write_error=None,
                   found=None, frames=None, camera_contrast=50, contrast_read_loss=0,
                   layout_found=None, capture_delays_ms=None):
        maix = types.ModuleType("maix")
        for name in ("app", "camera", "display", "err", "gpio", "image", "pinmap", "uart"):
            setattr(maix, name, Mock())

        def set_pin_function(pin, function):
            if pin == "B25" and function not in ("GPIOB25", "PWM6"):
                raise ValueError("func {} for pin B25 not valid".format(function))
            return 0

        def open_gpio(name, mode):
            if name != "GPIOB25":
                raise ValueError("invalid illumination GPIO name: {}".format(name))
            return maix.gpio.GPIO.return_value

        maix.pinmap.set_pin_function.side_effect = set_pin_function
        maix.gpio.GPIO.side_effect = open_gpio
        clock = iter(times)
        current_time = [0]

        def need_exit():
            try:
                current_time[0] = next(clock)
                return False
            except StopIteration:
                return True

        maix.app.need_exit.side_effect = need_exit
        current_contrast = [camera_contrast]

        def constrast(value=-1):
            if value >= 0:
                current_contrast[0] = value
            return current_contrast[0] - contrast_read_loss

        maix.camera.Camera.return_value.constrast.side_effect = constrast
        if frames is not None:
            maix.camera.Camera.return_value.read.side_effect = frames
        if capture_delays_ms is not None:
            delays = iter(capture_delays_ms)
            captures = iter(frames) if frames is not None else None
            def capture():
                current_time[0] += next(delays) / 1000
                return next(captures) if captures is not None else maix.camera.Camera.return_value.read.return_value
            maix.camera.Camera.return_value.read.side_effect = capture
        serial = maix.uart.UART.return_value
        serial.read.side_effect = incoming
        self.tx_times = []
        def write_response(response):
            self.tx_times.append(current_time[0])
            return len(response)
        serial.write.side_effect = write_error or write_response
        path = pathlib.Path(__file__).with_name("main.py")
        spec = importlib.util.spec_from_file_location("vision_entry_test", path)
        entry = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"maix": maix}):
            spec.loader.exec_module(entry)
        stable = Mock()
        stable.samples = []
        stable.started_ms = None
        stable.update.side_effect = confirmations
        stable.center.return_value = [164, 121]
        window_patch = (patch.object(entry, "StableWindow", wraps=entry.StableWindow)
                        if confirmations is None else
                        patch.object(entry, "StableWindow", return_value=stable))
        material_found = found["material"] if isinstance(found, dict) else found
        ring_found = found["ring"] if isinstance(found, dict) else found
        material_args = ({"side_effect": material_found} if isinstance(material_found, list)
                         else {"return_value": material_found})
        ring_args = ({"side_effect": ring_found} if isinstance(ring_found, list)
                     else {"return_value": ring_found})
        layout_args = ({"side_effect": layout_found} if isinstance(layout_found, list)
                       else {"return_value": layout_found})
        with patch.object(entry.time, "monotonic", side_effect=lambda: current_time[0]), \
                window_patch as constructor, \
                patch.object(entry, "find_material", **material_args) as material_detector, \
                patch.object(entry, "find_ring", **ring_args) as ring_detector, \
                patch.object(entry, "find_material_layout", **layout_args) as layout_detector, \
                patch("builtins.print") as output:
            if write_error:
                with self.assertRaises(RuntimeError):
                    entry.main()
            else:
                entry.main()
        self.window_calls = constructor.call_args_list
        self.stability_updates = stable.update.call_count
        self.layout_calls = layout_detector.call_count
        self.material_calls = material_detector.call_args_list
        self.ring_calls = ring_detector.call_args_list
        self.logs = [call.args[0] for call in output.call_args_list]
        return maix

    def request(self, mode, token, target=1, selector=1, mask=0):
        return encode(KIND_REQUEST, token, mode, selector, target, 0,
                      [0, 0, 320, 240, 160, 120, mask, 0])

    def tracking_request(self, token=9, mode=MODE_RING, selector=0, target=2, action=1):
        return encode(KIND_REQUEST, token, mode, selector, target, FLAG_TRACKING,
                      ([0] * 6 if action == -1 else [0, 0, 320, 240, 160, 120]) + [0, action])

    def tracking_responses(self, maix):
        return [Parser().feed(item.args[0])[0]
                for item in maix.uart.UART.return_value.write.call_args_list]

    def test_tracking_streams_raw_drifting_centers_with_rate_limit_and_new_frame_sequences(self):
        centers = [(150 + index * 4, 100 + index * 3, 80, 40, 40) for index in range(8)]
        maix = self.run_camera([self.tracking_request()] + [b""] * 9,
                               [index * 0.04 for index in range(10)], confirmations=[], found=centers)
        packets = self.tracking_responses(maix)
        self.assertEqual([1, 3, 5, 7], [packet["values"][6] for packet in packets])
        self.assertEqual([list(centers[index][:2]) for index in (0, 2, 4, 6)],
                         [packet["values"][:2] for packet in packets])
        self.assertTrue(all(packet["flags"] == FLAG_TRACKING | FLAG_VALID for packet in packets))
        self.assertTrue(all(packet["token"] == 9 for packet in packets))
        self.assertTrue(all(b - a >= 0.0499 for a, b in zip(self.tx_times, self.tx_times[1:])))
        self.assertEqual(1, len(self.window_calls))
        self.assertEqual(0, self.stability_updates)
        self.assertFalse(any(line.startswith("VISION RING") for line in self.logs))

    def test_tracking_at_thirteen_fps_sends_every_frame_instead_of_halving_rate(self):
        maix = self.run_camera([self.tracking_request()] + [b""] * 7,
                               [index / 13 for index in range(8)], confirmations=[],
                               found=(150, 100, 80, 70, 70), capture_delays_ms=[77] * 8)
        packets = self.tracking_responses(maix)
        self.assertEqual(list(range(1, 7)), [packet["values"][6] for packet in packets])
        self.assertTrue(all(packet["values"][7] == 77 for packet in packets))
        self.assertEqual(1, sum(line.startswith("VISION TRACK TX") for line in self.logs))
        self.assertIn("VISION APP version=1.0.24 ring_tracking=contours", self.logs)
        self.assertIn("version: 1.0.24", pathlib.Path(__file__).with_name("app.yaml").read_text(encoding="utf-8"))

    def test_tracking_ring_reference_comes_from_matching_stable_sample_and_advances(self):
        seed = (150, 100, 80, 70, 70)
        moved = (155, 103, 80, 70, 70)
        maix = self.run_camera(
            [self.request(MODE_RING, 2, 2, 0), b"", b"", self.tracking_request(),
             b"", b"", b"", b""],
            [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            confirmations=[False, False, True], found=[seed, moved, None, None])
        self.assertEqual([seed, moved, None],
                         [item.kwargs["reference"] for item in self.ring_calls[1:]])
        packets = self.tracking_responses(maix)
        self.assertEqual([FLAG_VALID | FLAG_STABLE, FLAG_TRACKING | FLAG_VALID,
                          FLAG_TRACKING, FLAG_TRACKING], [item["flags"] for item in packets])

    def test_tracking_ring_reference_rejects_expired_or_different_requests(self):
        seed = (150, 100, 80, 70, 70)
        variants = [self.tracking_request(),
                    self.tracking_request(selector=1),
                    encode(KIND_REQUEST, 9, MODE_RING, 1, 2,
                           FLAG_TRACKING | FLAG_RING_NO_DIGIT,
                           [0, 0, 320, 240, 160, 120, 0, 1]),
                    encode(KIND_REQUEST, 9, MODE_RING, 0, 2, FLAG_TRACKING,
                           [10, 0, 310, 240, 160, 120, 0, 1])]
        for index, tracking in enumerate(variants):
            with self.subTest(index=index):
                start = 2.1 if index == 0 else 0.3
                self.run_camera([self.request(MODE_RING, 2, 2, 1 if index == 2 else 0), b"", b"",
                                 tracking, b"", b""],
                                [0, 0.1, 0.2, start, start + 0.1, start + 0.2],
                                confirmations=[False, False, True], found=[seed, None])
                self.assertIsNone(self.ring_calls[-1].kwargs["reference"])

    def test_tracking_ring_cannot_reuse_sample_for_second_stream(self):
        seed = (150, 100, 80, 70, 70)
        self.run_camera([self.request(MODE_RING, 2, 2, 0), b"", b"",
                         self.tracking_request(9), b"", b"", self.tracking_request(10),
                         self.tracking_request(9, action=-1), b""],
                        [index * 0.1 for index in range(9)],
                        confirmations=[False, False, True], found=[seed, seed, None])
        self.assertEqual([seed, None], [item.kwargs["reference"] for item in self.ring_calls[1:]])

    def test_tracking_ring_new_failed_sample_invalidates_previous_reference(self):
        seed = (150, 100, 80, 70, 70)
        self.run_camera([self.request(MODE_RING, 2, 2, 0), b"", b"",
                         self.request(MODE_RING, 3, 2, 0), b"", b"",
                         self.tracking_request(), b"", b""],
                        [index * 0.1 for index in range(9)],
                        confirmations=[False, False, True, False, False, True],
                        found=[seed, None, None])
        self.assertIsNone(self.ring_calls[-1].kwargs["reference"])

    def test_tracking_stacking_uses_320_resolution_and_preserves_material_color(self):
        maix = self.run_camera([self.tracking_request(mode=MODE_MATERIAL, selector=5)] + [b""] * 4,
                               [index * 0.1 for index in range(5)], confirmations=[],
                               found=(170, 124, 80, 30, 40, 5))
        maix.camera.Camera.return_value.set_resolution.assert_called_once_with(320, 240)
        self.assertTrue(all(item.kwargs == {"scale": 1} for item in self.material_calls))
        packets = self.tracking_responses(maix)
        self.assertEqual([1, 2, 3], [packet["values"][6] for packet in packets])
        self.assertTrue(all(packet["values"][:6] == [170, 124, 80, 30, 40, 5] for packet in packets))
        self.assertTrue(all(not packet["flags"] & FLAG_STABLE for packet in packets))

    def test_tracking_stacking_preserves_original_hundred_ms_rate_limit(self):
        maix = self.run_camera([self.tracking_request(mode=MODE_MATERIAL, selector=5)] + [b""] * 7,
                               [index * 0.04 for index in range(8)], confirmations=[],
                               found=(170, 124, 80, 30, 40, 5))
        self.assertEqual([1, 4], [item["values"][6] for item in self.tracking_responses(maix)])

    def test_tracking_missing_captures_do_not_replay_positions_or_advance_sequence(self):
        maix = self.run_camera([self.tracking_request()] + [b""] * 5,
                               [0, 0.04, 0.08, 0.2, 0.3, 0.4], confirmations=[],
                               frames=[Mock(), Mock(), Mock(), None, None, Mock()],
                               found=[(150, 100, 80, 40, 40), (170, 120, 80, 40, 40)])
        packets = self.tracking_responses(maix)
        self.assertEqual([1, 2], [packet["values"][6] for packet in packets])
        self.assertEqual([[150, 100], [170, 120]], [packet["values"][:2] for packet in packets])
        self.assertEqual([0.08, 0.4], self.tx_times)

    def test_tracking_new_frame_without_target_sends_invalid_and_clears_old_center(self):
        maix = self.run_camera([self.tracking_request()] + [b""] * 3,
                               [0, 0.1, 0.2, 0.31], confirmations=[],
                               found=[(150, 100, 80, 40, 40), None])
        packets = self.tracking_responses(maix)
        self.assertEqual([FLAG_TRACKING | FLAG_VALID, FLAG_TRACKING], [packet["flags"] for packet in packets])
        self.assertEqual([0, 0, 0, 0, 0, 0, 2, 0], packets[-1]["values"])

    def test_tracking_end_only_cancels_matching_token_mode_selector_and_target(self):
        incoming = [self.tracking_request(), b"", b"",
                    self.tracking_request(token=8, action=-1),
                    self.tracking_request(mode=MODE_MATERIAL, action=-1),
                    self.tracking_request(selector=1, action=-1),
                    self.tracking_request(target=1, action=-1),
                    self.tracking_request(action=-1), b""]
        maix = self.run_camera(incoming, [index * 0.11 for index in range(len(incoming))],
                               confirmations=[], found=(150, 100, 80, 40, 40))
        packets = self.tracking_responses(maix)
        self.assertEqual([1, 2, 3, 4, 5], [packet["values"][6] for packet in packets])
        self.assertTrue(all(packet["token"] == 9 for packet in packets))

    def test_tracking_new_start_resets_discard_and_sequence_and_old_end_cannot_cancel_it(self):
        incoming = [self.tracking_request(1), b"", b"", self.tracking_request(2), b"",
                    self.tracking_request(1, action=-1), b"", self.tracking_request(2, action=-1)]
        maix = self.run_camera(incoming, [0, 0.04, 0.08, 0.2, 0.24, 0.28, 0.4, 0.52],
                               confirmations=[], found=(150, 100, 80, 40, 40))
        packets = self.tracking_responses(maix)
        self.assertEqual([1, 2, 2], [packet["token"] for packet in packets])
        self.assertEqual([1, 1, 2], [packet["values"][6] for packet in packets])

    def test_tracking_normal_request_replaces_stream_and_retains_ordinary_stability(self):
        incoming = [self.tracking_request(1), b"", b"", self.request(MODE_RING, 2, 2, 0),
                    self.tracking_request(1, action=-1), b"", b""]
        maix = self.run_camera(incoming, [index * 0.11 for index in range(len(incoming))],
                               confirmations=[False, False, True], found=(150, 100, 80, 40, 40))
        packets = self.tracking_responses(maix)
        self.assertEqual([1, 2], [packet["token"] for packet in packets])
        self.assertEqual([FLAG_TRACKING | FLAG_VALID, FLAG_VALID | FLAG_STABLE],
                         [packet["flags"] for packet in packets])
        self.assertEqual(2, len(self.window_calls))

    def test_tracking_reports_capture_processing_age_and_saturates_signed_field(self):
        for delay, age in ((35, 35), (40000, 32767)):
            with self.subTest(delay=delay):
                maix = self.run_camera([self.tracking_request(), b"", b""], [0, 0.1, 0.2],
                                       confirmations=[], found=(150, 100, 80, 40, 40),
                                       capture_delays_ms=[0, 0, delay])
                self.assertEqual(age, self.tracking_responses(maix)[0]["values"][7])

    def test_tracking_short_uart_write_still_closes_uart_and_illumination(self):
        maix = self.run_camera([self.tracking_request(), b"", b""], [0, 0.1, 0.2],
                               confirmations=[], write_error=lambda response: len(response) - 1,
                               found=(150, 100, 80, 40, 40))
        maix.uart.UART.return_value.close.assert_called_once()
        self.assertEqual(call(0), maix.gpio.GPIO.return_value.value.call_args_list[-1])

    def test_pickup_result_preserves_frame_size_and_color_with_median_center(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 9, 0, 0, 42), b"", b""],
                               [0, 0.1, 0.6], [False, False, True],
                               found=(168, 124, 90, 50, 50, 4))
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual(30, len(response))
        packet = Parser().feed(response)[0]
        self.assertEqual([164, 121, 90, 50, 50, 4, 0, 0], packet["values"])
        self.assertEqual(0, packet["selector"])
        self.assertEqual({"count": 10, "tolerance": 8, "minimum_ms": 0,
                          "trend_tolerance": 2},
                         self.window_calls[-1].kwargs)

    def test_storage_material_keeps_original_stability_and_center(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 10, 2, 4), b"", b""],
                               [0, 0.1, 0.6], [False, False, True],
                               found=(168, 124, 90, 50, 50, 4))
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual([168, 124], Parser().feed(response)[0]["values"][:2])
        self.assertEqual({"count": 5, "tolerance": 2, "minimum_ms": 500,
                          "trend_tolerance": None},
                         self.window_calls[-1].kwargs)

    def test_stack_preparation_switches_before_travel_without_detection_or_reply(self):
        prepare = encode(KIND_REQUEST, 0, MODE_MATERIAL, 0, 2, FLAG_STACK_PREPARE,
                         [0, 0, 320, 240, 160, 120, 0, 0])
        maix = self.run_camera([self.request(MODE_RING, 1), prepare, b"", b""],
                               [0, 0.1, 1.5, 3.0])
        cam = maix.camera.Camera.return_value
        cam.set_resolution.assert_called_once_with(320, 240)
        self.assertEqual([call(), call(80), call(50)], cam.constrast.call_args_list)
        changes = [item for item in cam.mock_calls if item[0] in ("constrast", "set_resolution")]
        self.assertEqual([call.constrast(), call.constrast(80),
                          call.set_resolution(320, 240), call.constrast(50)], changes)
        self.assertEqual([], self.material_calls)
        maix.uart.UART.return_value.write.assert_not_called()

    def test_stack_request_uses_320_coordinates_and_does_not_resize_after_preparation(self):
        prepare = encode(KIND_REQUEST, 0, MODE_MATERIAL, 0, 2, FLAG_STACK_PREPARE,
                         [0, 0, 320, 240, 160, 120, 0, 0])
        maix = self.run_camera(
            [prepare, b"", self.request(MODE_MATERIAL, 9, 2, 5, 16)] + [b""] * 6,
            [0, 1.5, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.8],
            found=(168, 124, 80, 50, 50, 5))
        cam = maix.camera.Camera.return_value
        cam.set_resolution.assert_called_once_with(320, 240)
        self.assertTrue(self.material_calls)
        self.assertTrue(all(item.kwargs == {"scale": 1} for item in self.material_calls))
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual(30, len(response))
        packet = Parser().feed(response)[0]
        self.assertEqual((9, 3, [168, 124]), (packet["token"], packet["flags"], packet["values"][:2]))
        frame = cam.read.return_value
        frame.draw_cross.assert_any_call(160, 120, maix.image.COLOR_WHITE, 10, 1)
        frame.draw_rect.assert_any_call(0, 0, 320, 240, maix.image.COLOR_YELLOW, 1)
        frame.draw_cross.assert_any_call(168, 124, maix.image.COLOR_GREEN, 8, 2)

    def test_leaving_stacking_restores_640_for_material_and_ring(self):
        for mode, target in ((MODE_MATERIAL, 0), (MODE_RING, 2)):
            with self.subTest(mode=mode):
                maix = self.run_camera(
                    [self.request(MODE_MATERIAL, 1, 2, 5), self.request(mode, 2, target, 5)],
                    [0, 0.1])
                self.assertEqual([call(320, 240), call(640, 480)],
                                 maix.camera.Camera.return_value.set_resolution.call_args_list)

    def test_ring_contrast_persists_after_result_and_timeout_then_restores(self):
        maix = self.run_camera(
            [self.request(MODE_RING, 1), b"", self.request(MODE_RING, 2),
             b"", self.request(MODE_MATERIAL, 3), b""],
            [0, 0.1, 0.2, 1.3, 1.4, 1.5],
            [True, False, False, True])
        self.assertEqual([call(0), call(0)],
                         maix.gpio.GPIO.return_value.value.call_args_list)
        maix.gpio.GPIO.assert_called_once_with("GPIOB25", maix.gpio.Mode.OUT)
        maix.pinmap.set_pin_function.assert_any_call("B25", "GPIOB25")
        self.assertEqual([call(), call(80), call(50)],
                         maix.camera.Camera.return_value.constrast.call_args_list)
        self.assertEqual(3, maix.uart.UART.return_value.write.call_count)
        maix.uart.UART.return_value.close.assert_called_once()

    def test_exception_turns_off_ring_light(self):
        maix = self.run_camera([self.request(MODE_RING, 1)], [0], [True],
                               RuntimeError("UART failure"))
        self.assertEqual([call(0), call(0)],
                         maix.gpio.GPIO.return_value.value.call_args_list)
        maix.uart.UART.return_value.close.assert_called_once()

    def test_material_only_does_not_change_camera_contrast_or_enable_light(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 1)], [0], [False])
        maix.camera.Camera.return_value.constrast.assert_not_called()
        maix.gpio.GPIO.assert_called_once_with("GPIOB25", maix.gpio.Mode.OUT)
        self.assertEqual([call(0), call(0)],
                         maix.gpio.GPIO.return_value.value.call_args_list)

    def test_material_contrast_does_not_drift_when_reentering_ring_mode(self):
        maix = self.run_camera(
            [self.request(MODE_RING, 1), self.request(MODE_MATERIAL, 2),
             self.request(MODE_RING, 3), self.request(MODE_MATERIAL, 4)],
            [0, 0.1, 0.2, 0.3], camera_contrast=50, contrast_read_loss=1)
        self.assertEqual([call(), call(80), call(49), call(80), call(49)],
                         maix.camera.Camera.return_value.constrast.call_args_list)

    def test_direct_stacking_restores_material_contrast_before_capture(self):
        for native, loss in ((50, 0), (37, 0), (50, 1)):
            with self.subTest(native=native, read_loss=loss):
                maix = self.run_camera(
                    [self.request(MODE_MATERIAL, 1, 0, 5),
                     self.request(MODE_RING, 2, 2, 0),
                     self.request(MODE_MATERIAL, 3, 2, 5, 16)],
                    [0, 0.1, 0.2], camera_contrast=native, contrast_read_loss=loss)
                cam = maix.camera.Camera.return_value
                baseline = native - loss
                self.assertEqual([call(), call(80), call(baseline)],
                                 cam.constrast.call_args_list)
                captures = [index for index, item in enumerate(cam.mock_calls) if item == call.read()]
                self.assertLess(cam.mock_calls.index(call.constrast(baseline)), captures[2])
                self.assertIn("VISION CAMERA mode=1 no_digit=0 contrast_set={} contrast_readback={}".format(
                    baseline, baseline - loss), self.logs)

    def test_second_round_material_fallback_reuses_first_material_contrast(self):
        stack_request = encode(KIND_REQUEST, 3, MODE_RING, 5, 2, FLAG_RING_NO_DIGIT,
                               [0, 0, 320, 240, 160, 120, 0, 0])
        maix = self.run_camera(
            [self.request(MODE_RING, 1, 2, 0), self.request(MODE_MATERIAL, 2, 0, 5),
             stack_request, self.request(MODE_MATERIAL, 4, 2, 5)],
            [0, 0.1, 0.2, 0.3], camera_contrast=50, contrast_read_loss=1)
        self.assertEqual([call(), call(80), call(49), call(49), call(49)],
                         maix.camera.Camera.return_value.constrast.call_args_list)
        self.assertEqual({"count": 5, "tolerance": 2, "minimum_ms": 500,
                          "trend_tolerance": None}, self.window_calls[-1].kwargs)

    def test_stack_ring_uses_material_contrast_before_capture_and_returns_valid_result(self):
        request = encode(KIND_REQUEST, 9, MODE_RING, 5, 2, FLAG_RING_NO_DIGIT,
                         [0, 0, 320, 240, 160, 120, 0, 0])
        maix = self.run_camera([request] + [b""] * 5,
                               [0, 0.05, 0.1, 0.2, 0.3, 0.4],
                               found=(160, 120, 80, 80, 80), camera_contrast=37)
        cam = maix.camera.Camera.return_value
        self.assertEqual([call(), call(37)], cam.constrast.call_args_list)
        self.assertEqual([call.constrast(), call.constrast(37), call.read()], cam.mock_calls[:3])
        packet = Parser().feed(maix.uart.UART.return_value.write.call_args.args[0])[0]
        self.assertEqual(3, packet["flags"])
        self.assertEqual(5, packet["selector"])
        self.assertEqual([160, 120], packet["values"][:2])

    def test_ring_variants_keep_640_until_stacking_material_request(self):
        for first_flags in (0, FLAG_RING_NO_DIGIT):
            with self.subTest(first_flags=first_flags):
                other_flags = first_flags ^ FLAG_RING_NO_DIGIT
                packets = [encode(KIND_REQUEST, token, MODE_RING, 5, 2, flags,
                                  [0, 0, 320, 240, 160, 120, 0, 0])
                           for token, flags in enumerate(
                               (first_flags, other_flags, other_flags, first_flags), 1)]
                packets.append(self.request(MODE_MATERIAL, 5, 2, 5))
                maix = self.run_camera(packets, [0, 0.1, 0.2, 0.3, 0.4], camera_contrast=37)
                cam = maix.camera.Camera.return_value
                first, other = (37, 80) if first_flags else (80, 37)
                self.assertEqual([call(), call(first), call(other), call(first), call(37)],
                                 cam.constrast.call_args_list)
                maix.camera.Camera.assert_called_once_with(640, 480)
                cam.set_resolution.assert_called_once_with(320, 240)
                captures = [index for index, item in enumerate(cam.mock_calls) if item == call.read()]
                self.assertGreater(cam.mock_calls.index(call.set_resolution(320, 240)), captures[3])

    def test_ring_resolution_switch_preserves_protocol_and_scales_overlay(self):
        maix = self.run_camera(
            [self.request(MODE_RING, 1), b"", b"", self.request(MODE_MATERIAL, 2)],
            [0, 0.1, 0.2, 0.3], [False, False, True, False],
            found=(168, 124, 80, 50, 50))
        cam = maix.camera.Camera.return_value
        maix.camera.Camera.assert_called_once_with(640, 480)
        cam.set_resolution.assert_not_called()
        self.assertEqual([call.constrast(), call.constrast(80), call.constrast(50)],
                         [item for item in cam.mock_calls
                          if item[0] in ("constrast", "set_resolution")])
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual([168, 124, 80, 50, 50, 0, 0, 0],
                         Parser().feed(response)[0]["values"])
        frame = cam.read.return_value
        frame.draw_rect.assert_any_call(0, 0, 640, 480, maix.image.COLOR_YELLOW, 1)
        frame.draw_cross.assert_any_call(336, 248, maix.image.COLOR_GREEN, 8, 2)

    def test_material_quality_is_diagnostic_only(self):
        for color in range(1, 7):
            for quality in (0, 29, 59, 100):
                with self.subTest(color=color, quality=quality):
                    maix = self.run_camera(
                        [self.request(MODE_MATERIAL, 1, 0, color), b"", b""],
                        [0, 0.1, 0.6], [False, False, True],
                        found=(168, 124, quality, 50, 50, color))
                    response = maix.uart.UART.return_value.write.call_args.args[0]
                    packet = Parser().feed(response)[0]
                    self.assertEqual(3, packet["flags"])
                    self.assertEqual(quality, packet["values"][2])

    def test_pickup_confirms_ten_frames_and_reports_offsets_fps(self):
        points = [(u, 110, 0, 50, 50, 4) for u in (164, 168, 166, 165, 164) * 2]
        maix = self.run_camera([self.request(MODE_MATERIAL, 9, 0, 4)] + [b""] * 11,
                               [index * 0.04 for index in range(12)], found=points)
        serial = maix.uart.UART.return_value
        self.assertEqual(1, serial.write.call_count)
        packet = Parser().feed(serial.write.call_args.args[0])[0]
        self.assertEqual(3, packet["flags"])
        self.assertEqual([165, 110], packet["values"][:2])
        maix.uart.UART.assert_called_once_with("/dev/ttyS4", 9600)
        maix.pinmap.set_pin_function.assert_any_call("A21", "UART4_TX")
        maix.pinmap.set_pin_function.assert_any_call("A22", "UART4_RX")
        frame = maix.camera.Camera.return_value.read.return_value
        frame.draw_string.assert_any_call(2, 2, "green center=(330,220) still=1",
                                          maix.image.COLOR_WHITE)
        frame.draw_string.assert_any_call(2, 20, "dx=10 dy=20 d=22.4px",
                                          maix.image.COLOR_WHITE)
        self.assertTrue(any("fps=25.0 elapsed=440ms" in line for line in self.logs))

    def test_pickup_does_not_confirm_with_only_nine_valid_frames(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 9, 0, 4)] + [b""] * 10,
                               [index * 0.04 for index in range(11)],
                               found=(160, 120, 0, 50, 50, 4))
        maix.uart.UART.return_value.write.assert_not_called()

    def test_pickup_drift_is_not_reported_as_stable(self):
        for axis in (0, 1):
            with self.subTest(axis=axis):
                points = []
                for shift in (0, 1, 2, 3, 4, 4, 5, 6, 7, 8):
                    point = [160, 120, 0, 50, 50, 4]
                    point[axis] += shift
                    points.append(tuple(point))
                maix = self.run_camera(
                    [self.request(MODE_MATERIAL, 9, 0, 4)] + [b""] * 11,
                    [index * 0.04 for index in range(12)], found=points)
                maix.uart.UART.return_value.write.assert_not_called()

    def test_pickup_confirms_again_after_drift_stops(self):
        points = [(u, 120, 0, 50, 50, 4)
                  for u in (160, 161, 162, 163, 164, 164, 165, 166, 167, 168)]
        points += [(168, 120, 0, 50, 50, 4)] * 9
        maix = self.run_camera(
            [self.request(MODE_MATERIAL, 9, 0, 4)] + [b""] * 20,
            [index * 0.04 for index in range(21)], found=points)
        serial = maix.uart.UART.return_value
        self.assertEqual(1, serial.write.call_count)
        packet = Parser().feed(serial.write.call_args.args[0])[0]
        self.assertEqual(3, packet["flags"])
        self.assertEqual([168, 120], packet["values"][:2])
        self.assertTrue(any("elapsed=800ms" in line for line in self.logs))

    def test_missing_capture_resets_pickup_window(self):
        frame = Mock()
        maix = self.run_camera([self.request(MODE_MATERIAL, 9, 0, 4)] + [b""] * 21,
                               [index * 0.01 for index in range(22)],
                               found=(160, 120, 0, 50, 50, 4),
                               frames=[frame] * 11 + [None] + [frame] * 10)
        self.assertEqual(1, maix.uart.UART.return_value.write.call_count)
        self.assertTrue(any("elapsed=210ms" in line for line in self.logs))

    def test_new_request_cannot_reuse_previous_pickup_samples(self):
        maix = self.run_camera(
            [self.request(MODE_MATERIAL, 9, 0, 4)] + [b""] * 10 +
            [self.request(MODE_MATERIAL, 10, 0, 4)] + [b""] * 10,
            [index * 0.01 for index in range(22)], found=(160, 120, 0, 50, 50, 4))
        maix.uart.UART.return_value.write.assert_not_called()

    def test_missing_pickup_capture_times_out_with_invalid_result(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 9, 0, 4), b""],
                               [0, 1.2], frames=[None, None])
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual(0, Parser().feed(response)[0]["flags"])

    def test_material_loss_and_color_switch_restart_ten_frame_count(self):
        green = (160, 120, 0, 50, 50, 4)
        yellow = (160, 120, 0, 50, 50, 2)
        for interruption in (None, yellow):
            with self.subTest(interruption=interruption):
                maix = self.run_camera(
                    [self.request(MODE_MATERIAL, 9, 0, 0)] + [b""] * 21,
                    [index * 0.01 for index in range(22)],
                    found=[green] * 9 + [interruption] + [green] * 10)
                self.assertEqual(1, maix.uart.UART.return_value.write.call_count)
                self.assertTrue(any("elapsed=210ms" in line for line in self.logs))

    def test_ring_accepts_three_stable_frames_but_not_two(self):
        for flags, selector in [(0, 0), (FLAG_RING_NO_DIGIT, 5)]:
            for count in (2, 3):
                with self.subTest(flags=flags, count=count):
                    request = encode(KIND_REQUEST, 9, MODE_RING, selector, 2, flags,
                                     [0, 0, 320, 240, 160, 120, 0, 0])
                    times = [0, 0.05] + [0.1, 0.3, 0.5, 0.7][:count]
                    maix = self.run_camera([request] + [b""] * (len(times) - 1), times,
                                           found=(160, 120, 80, 50, 50))
                    if count == 2:
                        maix.uart.UART.return_value.write.assert_not_called()
                    else:
                        response = maix.uart.UART.return_value.write.call_args.args[0]
                        self.assertEqual(3, Parser().feed(response)[0]["flags"])
                    self.assertEqual({"count": 3, "tolerance": 2, "minimum_ms": 180,
                                      "trend_tolerance": None}, self.window_calls[-1].kwargs)

    def test_three_ring_frames_still_require_180ms(self):
        for final_time, accepted in [(0.279, False), (0.280, True)]:
            with self.subTest(final_time=final_time):
                maix = self.run_camera(
                    [self.request(MODE_RING, 9, 2, 0)] + [b""] * 4,
                    [0, 0.05, 0.1, 0.2, final_time],
                    found=(160, 120, 80, 50, 50))
                self.assertEqual(int(accepted), maix.uart.UART.return_value.write.call_count)

    def test_ring_loss_or_jitter_restarts_three_frame_window(self):
        point = (160, 120, 80, 50, 50)
        shifted = (163, 120, 80, 50, 50)
        for interruption, final_point in [(None, point), (shifted, shifted)]:
            with self.subTest(interruption=interruption):
                found = [point] * 2 + [interruption] + [final_point] * 3
                maix = self.run_camera(
                    [self.request(MODE_RING, 9, 2, 0)] + [b""] * 7,
                    [0, 0.05] + [index * 0.1 for index in range(1, 7)], found=found)
                maix.uart.UART.return_value.write.assert_called_once()
                packet = Parser().feed(maix.uart.UART.return_value.write.call_args.args[0])[0]
                self.assertEqual(3, packet["flags"])
                self.assertEqual(list(final_point[:2]), packet["values"][:2])
                elapsed_ms = 600 if interruption is None else 500
                self.assertTrue(any("window=3" in line and "elapsed_ms={}".format(elapsed_ms) in line
                                    for line in self.logs))

    def test_three_stable_ring_frames_confirm_on_timeout_iteration(self):
        maix = self.run_camera([self.request(MODE_RING, 9, 2, 0)] + [b""] * 4,
                               [0, 0.05, 0.12, 0.45, 1.1],
                               found=(152, 122, 80, 80, 80))
        maix.uart.UART.return_value.write.assert_called_once()
        packet = Parser().feed(maix.uart.UART.return_value.write.call_args.args[0])[0]
        self.assertEqual(3, packet["flags"])
        self.assertEqual([152, 122], packet["values"][:2])

    def test_stack_ring_timeout_then_material_restores_resolution_and_contrast(self):
        ring_request = encode(KIND_REQUEST, 9, MODE_RING, 5, 2, FLAG_RING_NO_DIGIT,
                              [0, 0, 320, 240, 160, 120, 0, 0])
        maix = self.run_camera(
            [ring_request, b"", b"", self.request(MODE_MATERIAL, 10, 2, 5, 16)] + [b""] * 6,
            [0, 0.1, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8],
            found={"ring": [None], "material": [(160, 120, 0, 50, 50, 5)] * 5})
        serial = maix.uart.UART.return_value
        packets = [Parser().feed(call.args[0])[0] for call in serial.write.call_args_list]
        self.assertEqual([0, 3], [packet["flags"] for packet in packets])
        self.assertEqual([9, 10], [packet["token"] for packet in packets])
        self.assertEqual([MODE_RING, MODE_MATERIAL], [packet["mode"] for packet in packets])
        self.assertEqual([5, 5], [packet["selector"] for packet in packets])
        maix.camera.Camera.assert_called_once_with(640, 480)
        maix.camera.Camera.return_value.set_resolution.assert_called_once_with(320, 240)
        self.assertEqual([call(0), call(0)],
                         maix.gpio.GPIO.return_value.value.call_args_list)
        self.assertEqual([call(), call(50), call(50)],
                         maix.camera.Camera.return_value.constrast.call_args_list)


if __name__ == "__main__":
    unittest.main()

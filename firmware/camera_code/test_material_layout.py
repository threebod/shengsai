import pathlib
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import vision_settings as settings
from material_layout import LayoutMemory, LayoutWindow, find_material_layout
import test_vision_main as vision_main_tests
from vision_protocol import (FLAG_COLOR_INFERRED, FLAG_LAYOUT_MOVING, FLAG_LAYOUT_REFERENCE, FLAG_MATERIAL_LAYOUT, FLAG_STABLE,
                             FLAG_VALID, KIND_REQUEST, LAYOUT_CONFIRM, LAYOUT_END, LAYOUT_LOCATE,
                             LAYOUT_TRACK, MODE_MATERIAL, Parser, encode, validate_request)


def request(token=1, phase=LAYOUT_CONFIRM, mask=13, selector=0):
    return encode(KIND_REQUEST, token, MODE_MATERIAL, selector, 0, FLAG_MATERIAL_LAYOUT,
                  [0, 0, 320, 240, 160, 205, mask, phase])


def layout(colors=(1, 4, 3), missing=None, bottom=(160, 200)):
    points = [(bottom[0], bottom[1], 80, 40, 40, colors[0]),
              (70, 50, 80, 40, 40, colors[1]), (250, 50, 80, 40, 40, colors[2])]
    if missing is not None:
        points[missing] = None
    return {"points": tuple(points), "colors": colors, "inferred": missing is not None}


class MaterialLayoutDetectorTest(unittest.TestCase):
    def blob(self, u, v, area=6400, width=80, height=80):
        blob = Mock()
        for name, value in (("cx", u), ("cy", v), ("area", area), ("pixels", area),
                            ("x", u - width // 2), ("y", v - height // 2),
                            ("w", width), ("h", height)):
            getattr(blob, name).return_value = value
        return blob

    def detect(self, blobs, phase=LAYOUT_CONFIRM, mask=13, selector=0):
        frame = Mock()
        frame.find_blobs.side_effect = lambda thresholds, **kwargs: blobs.get(
            next(color for color, threshold in settings.COLOR_THRESHOLDS.items()
                 if thresholds == [threshold]), [])
        result = find_material_layout(frame, Parser().feed(request(
            phase=phase, mask=mask, selector=selector))[0])
        for call in frame.find_blobs.call_args_list:
            self.assertEqual((0, 0, 640, 480), call.kwargs["roi"])
        return result

    def test_bottom_is_lowest_not_largest_and_coordinates_are_scaled(self):
        result = self.detect({1: [self.blob(320, 400)],
                              4: [self.blob(140, 100, 20000)], 3: [self.blob(500, 100)]})
        self.assertEqual((160, 200), result["points"][0][:2])
        self.assertEqual((1, 4, 3), result["colors"])
        self.assertFalse(result["inferred"])

    def test_two_known_colors_infer_remaining_upper_slot(self):
        result = self.detect({1: [self.blob(320, 400)], 4: [self.blob(140, 100)]})
        self.assertEqual((1, 4, 3), result["colors"])
        self.assertIsNone(result["points"][2])
        self.assertTrue(result["inferred"])

    def test_missing_bottom_can_infer_color_but_never_center_or_stability(self):
        result = self.detect({4: [self.blob(140, 100)], 3: [self.blob(500, 100)]})
        self.assertEqual((1, 4, 3), result["colors"])
        self.assertIsNone(result["points"][0])
        window = LayoutWindow()
        for _ in range(20):
            self.assertFalse(window.update(result))

    def test_position_conflicts_are_rejected(self):
        for blobs in ({1: [self.blob(320, 400)], 4: [self.blob(140, 300)]},
                      {1: [self.blob(320, 400)], 4: [self.blob(140, 100)],
                       3: [self.blob(240, 100)]}):
            self.assertIsNone(self.detect(blobs))

    def test_single_entry_target_can_align_from_either_half_without_inference(self):
        for u, v in ((320, 400), (220, 100), (500, 280)):
            result = self.detect({1: [self.blob(u, v)]})
            self.assertEqual((round(u / 2), round(v / 2)), result["points"][0][:2])
            self.assertEqual((1, 0, 0), result["colors"])
            self.assertEqual((None, None), result["points"][1:])
            self.assertFalse(result["inferred"])
        self.assertIsNone(self.detect({}))

    def test_small_upper_materials_use_reference_area_in_camera_pixels(self):
        result = self.detect({1: [self.blob(320, 400)],
                              4: [self.blob(30, 40, 2500, 50, 50)],
                              3: [self.blob(610, 40, 2500, 50, 50)]})
        self.assertEqual((1, 4, 3), result["colors"])
        self.assertTrue(all(result["points"]))
        self.assertFalse(result["inferred"])

    def test_reference_area_limits_are_inclusive_and_black_has_own_minimum(self):
        for color, minimum in ((1, 1500), (5, 4000)):
            for area, accepted in ((minimum - 1, False), (minimum, True),
                                    (30000, True), (30001, False)):
                with self.subTest(color=color, area=area):
                    result = self.detect({color: [self.blob(320, 400, area)]},
                                         mask=1 << (color - 1), selector=color)
                    self.assertEqual(accepted, result is not None)

    def test_initial_location_retains_all_real_centers_for_display_only(self):
        for phase in (LAYOUT_TRACK,):
            result = self.detect({1: [self.blob(320, 410)],
                                  4: [self.blob(140, 100)], 3: [self.blob(500, 100)]},
                                 phase=phase)
            self.assertEqual((1, 0, 0), result["colors"])
            self.assertEqual((None, None), result["points"][1:])
            self.assertEqual({(160, 205), (70, 50), (250, 50)},
                             {point[:2] for point in result["visible_points"]})

    def test_initial_location_reports_layout_in_the_same_six_frame_window(self):
        result = self.detect({1: [self.blob(320, 410)],
                              4: [self.blob(140, 100)], 3: [self.blob(500, 100)]},
                             phase=LAYOUT_LOCATE)
        self.assertEqual((1, 4, 3), result["colors"])
        window = LayoutWindow()
        for index in range(6):
            self.assertEqual(index == 5, window.update(result))
        self.assertEqual([160, 205], window.center())

    def test_each_color_checks_area_after_selecting_largest_blob_like_reference(self):
        result = self.detect({1: [self.blob(200, 300, 6000), self.blob(320, 400)],
                              4: [self.blob(140, 100)]})
        self.assertEqual((160, 200), result["points"][0][:2])
        result = self.detect({1: [self.blob(320, 400), self.blob(300, 380, 30001)]})
        self.assertIsNone(result)

    def test_larger_overlapping_color_wins_regardless_of_color_number(self):
        for large_color, small_color in ((1, 4), (4, 1)):
            result = self.detect({large_color: [self.blob(320, 400, 8000)],
                                  small_color: [self.blob(330, 400, 6400)],
                                  3: [self.blob(500, 100)]})
            self.assertEqual(large_color, result["colors"][0])
            self.assertIsNone(result["points"][1])
            self.assertTrue(result["inferred"])

    def test_touching_boxes_survive_but_one_pixel_overlap_is_suppressed(self):
        for distance, visible in ((80, True), (79, False)):
            result = self.detect({1: [self.blob(320, 400, 8000)],
                                  4: [self.blob(320 + distance, 400)]},
                                 mask=9, selector=1)
            self.assertEqual(visible, result is None)  # Two lower slots conflict only if both survive.

    def test_identical_and_nested_boxes_have_only_one_actual_color(self):
        for width, height in ((80, 80), (70, 70)):
            result = self.detect({1: [self.blob(320, 400, 8000)],
                                  4: [self.blob(320, 400, 6400, width, height)]})
            self.assertEqual((1, 0, 0), result["colors"])
            self.assertEqual((None, None), result["points"][1:])
            self.assertFalse(result["inferred"])

    def test_upper_rectangles_are_accepted_without_aspect_filter_like_reference(self):
        result = self.detect({1: [self.blob(320, 400)],
                              4: [self.blob(140, 100, 7200, width=120, height=60)],
                              3: [self.blob(500, 100, 7200, width=60, height=120)]})
        self.assertEqual((1, 4, 3), result["colors"])
        self.assertTrue(all(result["points"]))
        self.assertFalse(result["inferred"])

    def test_initial_location_tracks_lowest_before_two_objects_reach_layout_slots(self):
        for phase in (LAYOUT_LOCATE, LAYOUT_TRACK):
            result = self.detect({1: [self.blob(300, 160)], 4: [self.blob(180, 60)]}, phase=phase)
            self.assertEqual((150, 80), result["points"][0][:2])
            self.assertEqual((1, 0, 0), result["colors"])
            self.assertFalse(result["inferred"])

    def test_layout_requests_require_three_colors_and_known_phase(self):
        for mask, phase, valid in ((13, 0, True), (13, 1, True), (13, 2, True),
                                   (0, 1, False), (1, 1, False), (63, 1, False),
                                   (13, 3, True), (13, 4, False)):
            self.assertEqual(valid, validate_request(Parser().feed(request(1, phase, mask))[0]))

    def test_recheck_accepts_only_remaining_colors_and_requested_target(self):
        for mask, selector, phase, valid in ((12, 4, 1, True), (8, 4, 1, True),
                                            (12, 1, 1, False), (0, 4, 1, False),
                                            (63, 4, 1, False), (12, 4, 3, False)):
            self.assertEqual(valid, validate_request(Parser().feed(
                request(1, phase, mask, selector))[0]))

    def test_remaining_upper_targets_keep_slots_without_inventing_collected_bottom(self):
        for blobs, mask, colors in (({4: [self.blob(140, 100)], 3: [self.blob(500, 100)]},
                                    12, (0, 4, 3)),
                                   ({4: [self.blob(140, 100)]}, 8, (0, 4, 0))):
            result = self.detect(blobs, mask=mask, selector=4)
            self.assertEqual(colors, result["colors"])
            self.assertIsNone(result["points"][0])
            self.assertFalse(result["inferred"])
            window = LayoutWindow()
            for index in range(6):
                self.assertEqual(index == 5, window.update(result, 4))
            self.assertEqual([70, 50], window.center(4))

    def test_recheck_missing_target_or_rotating_slot_resets_window(self):
        result = self.detect({4: [self.blob(140, 100)]}, mask=12, selector=4)
        self.assertEqual((0, 4, 0), result["colors"])
        window = LayoutWindow()
        for _ in range(6):
            window.update(result, 4)
        missing = self.detect({3: [self.blob(500, 100)]}, mask=12, selector=4)
        self.assertFalse(window.update(missing, 4))
        rotated = self.detect({4: [self.blob(500, 100)]}, mask=8, selector=4)
        for index in range(6):
            self.assertEqual(index == 5, window.update(rotated, 4))
        self.assertEqual([250, 50], window.center(4))

    def test_window_needs_six_frames_and_one_still_material_despite_upper_motion(self):
        window = LayoutWindow()
        for index in range(6):
            self.assertEqual(index == 5, window.update(layout(missing=2)))
        self.assertEqual([160, 200], window.center())
        moving = layout(missing=2)
        moving["points"] = (moving["points"][0], (80, 50, 80, 40, 40, 4), None)
        self.assertTrue(window.update(moving))
        for _ in range(6):
            window.update(layout())
        self.assertFalse(window.update(layout(colors=(4, 1, 3))))
        self.assertFalse(window.update(None))

    def test_any_visible_slot_can_confirm_while_other_centers_move(self):
        for still_slot in range(3):
            window = LayoutWindow()
            for index in range(6):
                observed = layout()
                observed["points"] = tuple(
                    (point[0] + (0 if slot == still_slot else index * 3), *point[1:])
                    for slot, point in enumerate(observed["points"]))
                self.assertEqual(index == 5, window.update(observed))

    def test_still_requires_both_axes_on_the_same_material_and_revokes_on_motion(self):
        window = LayoutWindow()
        for _ in range(6):
            window.update(layout())
        moving = layout(bottom=(180, 200))
        moving["points"] = (moving["points"][0], (70, 70, 80, 40, 40, 4),
                            (270, 70, 80, 40, 40, 3))
        self.assertFalse(window.update(moving))
        for index in range(5):
            self.assertEqual(index == 4, window.update(moving))

    def test_motion_requires_full_observed_window_and_clears_on_loss_or_reset(self):
        window = LayoutWindow()
        for index in range(6):
            observed = layout()
            observed["points"] = tuple((point[0] + index * 3, *point[1:])
                                       for point in observed["points"])
            self.assertFalse(window.update(observed))
            self.assertEqual(index == 5, window.moving)
        window.update(None)
        self.assertFalse(window.moving)
        for index in range(6):
            self.assertEqual(index == 5, window.update(layout()))
            self.assertFalse(window.moving)
        window.reset()
        self.assertFalse(window.moving)

    def test_gradual_drift_and_new_confirmation_cannot_reuse_old_frames(self):
        window = LayoutWindow()
        for shift in range(10):
            observed = layout()
            observed["points"] = tuple((point[0] + shift, *point[1:]) for point in observed["points"])
            self.assertFalse(window.update(observed))
        for _ in range(6):
            window.update(layout())
        window.reset()
        for _ in range(5):
            self.assertFalse(window.update(layout()))

    def test_single_target_stability_does_not_transfer_to_color_confirmation(self):
        window = LayoutWindow()
        single = {"points": (layout()["points"][0], None, None),
                  "colors": (1, 0, 0), "inferred": False}
        for index in range(6):
            self.assertEqual(index == 5, window.update(single))
        for index in range(6):
            self.assertEqual(index == 5, window.update(layout(missing=2)))


class MaterialLayoutMemoryTest(unittest.TestCase):
    def test_ccw_turns_preserve_color_order_and_remove_collected_colors(self):
        memory = LayoutMemory()
        memory.remember(layout())
        packet = Parser().feed(request(mask=12, selector=4))[0]
        for colors in ((0, 4, 3), (4, 3, 0), (3, 0, 4)):
            visible = list(layout(colors=colors)["points"])
            visible = tuple(point if color else None for color, point in zip(colors, visible))
            detected = {"points": visible, "colors": colors, "inferred": False}
            resolved = memory.resolve(detected, packet)
            self.assertEqual(colors, resolved["colors"])
            self.assertFalse(resolved["inferred"])

    def test_visible_reference_infers_hidden_target_without_making_a_center(self):
        memory = LayoutMemory()
        memory.remember(layout())
        packet = Parser().feed(request(mask=12, selector=4))[0]
        detected = {"points": (None, (70, 50, 80, 40, 40, 3), None),
                    "colors": (0, 3, 0), "inferred": False}
        resolved = memory.resolve(detected, packet)
        self.assertEqual((4, 3, 0), resolved["colors"])
        self.assertIsNone(resolved["points"][0])
        self.assertEqual(1, resolved["reference"])
        self.assertTrue(resolved["inferred"])
        window = LayoutWindow()
        for index in range(6):
            self.assertEqual(index == 5, window.update(resolved, 4))
        self.assertEqual([70, 50], window.center(4))

    def test_no_memory_no_visible_reference_and_conflicting_order_cannot_infer(self):
        packet = Parser().feed(request(mask=12, selector=4))[0]
        detected = {"points": (None, (70, 50, 80, 40, 40, 3), None),
                    "colors": (0, 3, 0), "inferred": False}
        memory = LayoutMemory()
        self.assertIs(detected, memory.resolve(detected, packet))
        memory.remember(layout())
        self.assertIsNone(memory.resolve(None, packet))
        conflict = {"points": (None, (70, 50, 80, 40, 40, 3), (250, 50, 80, 40, 40, 4)),
                    "colors": (0, 3, 4), "inferred": False}
        self.assertIsNone(memory.resolve(conflict, packet))
        memory.reset()
        self.assertIs(detected, memory.resolve(detected, packet))

    def test_reference_center_can_shift_after_positioning_without_rechecking_old_anchor(self):
        memory = LayoutMemory()
        memory.remember(layout())
        packet = Parser().feed(request(mask=12, selector=4))[0]
        shifted = {"points": (None, (90, 75, 80, 40, 40, 3), None),
                   "colors": (0, 3, 0), "inferred": False}
        resolved = memory.resolve(shifted, packet)
        self.assertEqual((4, 3, 0), resolved["colors"])
        window = LayoutWindow()
        for index in range(6):
            self.assertEqual(index == 5, window.update(resolved, 4))
        self.assertEqual([90, 75], window.center(4))

    def test_initial_missing_slot_can_later_supply_a_real_stable_reference(self):
        memory = LayoutMemory()
        memory.remember(layout(missing=2))
        packet = Parser().feed(request(mask=12, selector=4))[0]
        detected = {"points": (None, None, (250, 50, 80, 40, 40, 3)),
                    "colors": (0, 0, 3), "inferred": False}
        resolved = memory.resolve(detected, packet)
        self.assertEqual((0, 4, 3), resolved["colors"])
        self.assertEqual(2, resolved["reference"])
        self.assertIsNone(resolved["points"][1])

    def test_inference_loses_stability_when_rotation_or_visibility_changes(self):
        memory = LayoutMemory()
        memory.remember(layout())
        packet = Parser().feed(request(mask=12, selector=4))[0]
        window = LayoutWindow()
        for slot in (1, 2):
            points = [None, None, None]
            points[slot] = (70 if slot == 1 else 250, 50, 80, 40, 40, 3)
            observed = {"points": tuple(points), "colors": (0, 0, 0), "inferred": False}
            resolved = memory.resolve(observed, packet)
            for index in range(6):
                self.assertEqual(index == 5, window.update(resolved, 4))
        self.assertFalse(window.update(None, 4))


class MaterialLayoutStreamTest(unittest.TestCase):
    run_camera = vision_main_tests.VisionMainTest.run_camera

    def responses(self, maix):
        return [Parser().feed(call.args[0])[0]
                for call in maix.uart.UART.return_value.write.call_args_list]

    def test_location_six_frames_confirm_layout_and_seed_recheck_memory(self):
        reference = {"points": (None, (70, 50, 80, 40, 40, 3), None),
                     "colors": (0, 3, 0), "inferred": False}
        maix = self.run_camera(
            [request(phase=LAYOUT_LOCATE)] + [b""] * 7 +
            [request(2, LAYOUT_END), request(3, mask=12, selector=4)] + [b""] * 8,
            [index * 0.1 for index in range(18)],
            layout_found=[layout()] * 6 + [reference] * 7)
        located = [row for row in self.responses(maix) if row["token"] == 1]
        self.assertFalse(any(row["flags"] & FLAG_STABLE for row in located[:-1]))
        self.assertTrue(located[-1]["flags"] & FLAG_STABLE)
        self.assertEqual([1, 4, 3], located[-1]["values"][5:8])
        rechecked = [row for row in self.responses(maix) if row["token"] == 3]
        self.assertTrue(rechecked[-1]["flags"] & FLAG_STABLE)
        self.assertTrue(rechecked[-1]["flags"] & FLAG_LAYOUT_REFERENCE)

    def test_memory_survives_pickup_end_but_new_confirmation_needs_six_frames(self):
        reference = {"points": (None, (90, 75, 80, 40, 40, 3), None),
                     "colors": (0, 3, 0), "inferred": False}
        maix = self.run_camera(
            [request()] + [b""] * 12 + [request(2, LAYOUT_END), request(3, mask=12, selector=4)] + [b""] * 14,
            [index * 0.04 for index in range(29)],
            layout_found=[layout()] * 11 + [reference] * 13)
        replies = [row for row in self.responses(maix) if row["token"] == 3]
        self.assertTrue(replies)
        self.assertFalse(replies[0]["flags"] & FLAG_STABLE)
        result = replies[-1]
        self.assertEqual(FLAG_MATERIAL_LAYOUT | FLAG_VALID | FLAG_STABLE |
                         FLAG_COLOR_INFERRED | FLAG_LAYOUT_REFERENCE, result["flags"])
        self.assertEqual([4, 3, 0], result["values"][5:8])
        self.assertEqual([90, 75], result["values"][:2])

    def test_stream_confirms_when_only_one_visible_material_is_still(self):
        frames = []
        for index in range(13):
            observed = layout(bottom=(160 + index * 3, 200))
            observed["points"] = (observed["points"][0], observed["points"][1],
                                  (250 - index * 3, 50, 80, 40, 40, 3))
            frames.append(observed)
        maix = self.run_camera([request()] + [b""] * 14,
                               [index * 0.04 for index in range(15)], layout_found=frames)
        result = self.responses(maix)[-1]
        self.assertEqual(FLAG_MATERIAL_LAYOUT | FLAG_VALID | FLAG_STABLE, result["flags"])
        self.assertEqual([1, 4, 3], result["values"][5:8])

    def test_new_round_discards_cached_colors_before_partial_recheck(self):
        single = {"points": (None, (70, 50, 80, 40, 40, 3), None),
                  "colors": (0, 3, 0), "inferred": False}
        maix = self.run_camera(
            [request()] + [b""] * 12 + [request(2, LAYOUT_END), request(3, LAYOUT_TRACK),
             request(4, LAYOUT_END), request(5, mask=12, selector=4)] + [b""] * 14,
            [index * 0.04 for index in range(31)],
            layout_found=[layout()] * 11 + [single] * 13)
        replies = [row for row in self.responses(maix) if row["token"] == 5]
        self.assertTrue(replies)
        self.assertFalse(any(row["flags"] & (FLAG_STABLE | FLAG_LAYOUT_REFERENCE) for row in replies))

    def test_recheck_stream_confirms_last_upper_target_at_its_actual_center(self):
        remaining = {"points": (None, (70, 50, 80, 40, 40, 4), None),
                     "colors": (0, 4, 0), "inferred": False}
        maix = self.run_camera([request(mask=8, selector=4)] + [b""] * 14,
                               [index * 0.04 for index in range(15)], layout_found=remaining)
        result = self.responses(maix)[-1]
        self.assertEqual(FLAG_MATERIAL_LAYOUT | FLAG_VALID | FLAG_STABLE, result["flags"])
        self.assertEqual(4, result["selector"])
        self.assertEqual([70, 50], result["values"][:2])
        self.assertEqual([0, 4, 0], result["values"][5:8])

    def test_stream_reports_real_motion_before_stopping_but_not_window_warmup(self):
        for moving, partial in ((False, False), (True, False), (False, True), (True, True)):
            with self.subTest(moving=moving, partial=partial):
                frames = []
                for index in range(6):
                    observed = layout()
                    observed["points"] = tuple((point[0] + (index * 3 if moving else 0), *point[1:])
                                               for point in observed["points"])
                    if partial:
                        observed["points"] = (observed["points"][0], None, None)
                        observed["colors"] = (1, 0, 0)
                    frames.append(observed)
                frames += [layout()] * 6
                maix = self.run_camera([request(phase=LAYOUT_LOCATE)] + [b""] * 13,
                                       [index * 0.11 for index in range(14)], layout_found=frames)
                replies = self.responses(maix)
                self.assertEqual(moving, any(row["flags"] & FLAG_LAYOUT_MOVING for row in replies))
                self.assertFalse(any(row["flags"] & FLAG_LAYOUT_MOVING for row in replies[:7]))
                self.assertTrue(replies[-1]["flags"] & FLAG_STABLE)
                self.assertFalse(replies[-1]["flags"] & FLAG_LAYOUT_MOVING)

    def test_stream_keeps_detecting_after_confirmation_and_limits_uart_rate(self):
        maix = self.run_camera([request()] + [b""] * 20,
                               [index * 0.04 for index in range(21)], layout_found=layout(missing=2))
        responses = self.responses(maix)
        self.assertEqual(19, self.layout_calls)
        self.assertTrue(any(row["flags"] & FLAG_STABLE for row in responses))
        self.assertTrue(all(row["flags"] & FLAG_COLOR_INFERRED for row in responses[1:]))
        self.assertTrue(all(b - a >= 0.0999 for a, b in zip(self.tx_times, self.tx_times[1:])))
        self.assertEqual([1, 4, 3], responses[-1]["values"][5:8])

    def test_tracking_never_triggers_stable_and_confirmation_starts_fresh(self):
        maix = self.run_camera([request(1, LAYOUT_TRACK)] + [b""] * 11 +
                               [request(2, LAYOUT_CONFIRM)] + [b""] * 4,
                               [index * 0.04 for index in range(17)], layout_found=layout())
        self.assertFalse(any(row["flags"] & FLAG_STABLE for row in self.responses(maix)))
        self.assertEqual(2, self.responses(maix)[-1]["token"])

    def test_lost_or_rotating_layout_revokes_ready(self):
        maix = self.run_camera([request()] + [b""] * 18,
                               [index * 0.04 for index in range(19)],
                               layout_found=[layout()] * 12 + [None] * 5)
        responses = self.responses(maix)
        self.assertTrue(any(row["flags"] & FLAG_STABLE for row in responses))
        self.assertFalse(responses[-1]["flags"] & (FLAG_VALID | FLAG_STABLE))

    def test_end_stops_stream_and_missing_capture_sends_invalid_heartbeat(self):
        maix = self.run_camera([request(), b"", request(2, LAYOUT_END), b""],
                               [0, 0.1, 0.2, 0.3], frames=[None] * 4)
        self.assertEqual(2, len(self.responses(maix)))
        self.assertTrue(all(row["flags"] == FLAG_MATERIAL_LAYOUT for row in self.responses(maix)))

    def test_missing_bottom_is_not_displayed_or_reported_as_a_real_center(self):
        maix = self.run_camera([request()] + [b""] * 14,
                               [index * 0.04 for index in range(15)], layout_found=layout(missing=0))
        self.assertTrue(all(not row["flags"] & (FLAG_VALID | FLAG_STABLE)
                            for row in self.responses(maix)))
        maix.camera.Camera.return_value.read.return_value.draw_string.assert_any_call(
            2, 20, "target=(320,410) bottom missing", maix.image.COLOR_WHITE)

    def test_single_entry_target_reports_stable_position_with_unknown_upper_colors(self):
        point = (110, 80, 80, 40, 40, 1)
        single = {"points": (point, None, None), "colors": (1, 0, 0), "inferred": False}
        maix = self.run_camera([request()] + [b""] * 14,
                               [index * 0.04 for index in range(15)], layout_found=single)
        result = self.responses(maix)[-1]
        self.assertEqual(FLAG_MATERIAL_LAYOUT | FLAG_VALID | FLAG_STABLE, result["flags"])
        self.assertEqual([110, 80], result["values"][:2])
        self.assertEqual([1, 0, 0], result["values"][5:8])

    def test_initial_location_displays_upper_centers_without_reporting_slot_colors(self):
        visible = layout()["points"]
        single = {"points": (visible[0], None, None), "colors": (1, 0, 0),
                  "inferred": False, "visible_points": visible}
        maix = self.run_camera([request(phase=LAYOUT_LOCATE)] + [b""] * 14,
                               [index * 0.04 for index in range(15)], layout_found=single)
        result = self.responses(maix)[-1]
        self.assertEqual([1, 0, 0], result["values"][5:8])
        self.assertEqual([160, 200], result["values"][:2])
        frame = maix.camera.Camera.return_value.read.return_value
        for point in visible:
            frame.draw_cross.assert_any_call(point[0] * 2, point[1] * 2,
                                              maix.image.COLOR_GREEN, 8, 2)
        frame.draw_string.assert_any_call(140, 100, "seen c=4", maix.image.COLOR_GREEN)
        frame.draw_string.assert_any_call(500, 100, "seen c=3", maix.image.COLOR_GREEN)

    def test_initial_position_phase_and_fresh_color_phase_have_independent_windows(self):
        single = {"points": (layout()["points"][0], None, None),
                  "colors": (1, 0, 0), "inferred": False}
        maix = self.run_camera([request(1, LAYOUT_LOCATE)] + [b""] * 12 +
                               [request(2, LAYOUT_CONFIRM)] + [b""] * 4,
                               [index * 0.04 for index in range(18)],
                               layout_found=[single] * 11 + [layout(missing=2)] * 9)
        responses = self.responses(maix)
        self.assertTrue(any(row["token"] == 1 and row["flags"] & FLAG_STABLE for row in responses))
        colors = [row for row in responses if row["token"] == 2]
        self.assertTrue(colors)
        self.assertTrue(all(not row["flags"] & FLAG_STABLE for row in colors))


if __name__ == "__main__":
    unittest.main()

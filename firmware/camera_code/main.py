"""MaixCAM vehicle alignment entry point. Deploy this directory together."""
import math
import time

from maix import app, camera, display, err, gpio, image, pinmap, uart

from vision_detectors import find_material, find_ring
from material_layout import LayoutMemory, LayoutWindow, find_material_layout
from vision_protocol import (FLAG_COLOR_INFERRED, FLAG_LAYOUT_REFERENCE, FLAG_MATERIAL_LAYOUT,
                             FLAG_RING_NO_DIGIT, FLAG_STACK_PREPARE, FLAG_STABLE, FLAG_VALID, KIND_RESULT,
                             FLAG_TRACKING, TRACKING_END,
                             LAYOUT_CONFIRM, LAYOUT_END, LAYOUT_LOCATE,
                             MODE_MATERIAL, MODE_RING,
                             Parser, StableWindow, encode, tracking_values, validate_request)
import vision_settings as settings


def main():
    print("VISION APP version=1.0.24 ring_tracking=contours")
    err.check_raise(pinmap.set_pin_function("B25", "GPIOB25"), "illumination GPIO mapping")
    illuminator = gpio.GPIO("GPIOB25", gpio.Mode.OUT)
    illuminator.value(0)
    for pin, function in settings.UART_PINS.items():
        err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping")
    serial = uart.UART(settings.UART_DEVICE, settings.BAUD)
    cam = camera.Camera(640, 480)
    scale = 2  # Camera pixels per 320x240 protocol pixel.
    screen = display.Display()
    parser = Parser()
    stable = StableWindow()
    layout_window = LayoutWindow()
    layout_memory = LayoutMemory()
    layout_last_tx = None
    tracking_last_tx = None
    tracking_sequence = 0
    tracking_log_at = None
    ring_seed = None
    tracking_ring = None
    request = None
    light_mode = MODE_MATERIAL
    ring_no_digit = False
    material_contrast = None
    request_started = 0
    discard_frames = 0
    detection_frames = 0
    detection_started = None
    ring_hits = 0
    ring_detect_ms = 0
    material_status = "material waiting"
    material_detail = ""
    material_metrics = ""
    ring_label = "ring 2"
    try:
        while not app.need_exit():
            now_ms = int(time.monotonic() * 1000)
            incoming = serial.read(-1, 0) or b""
            if incoming:
                print("VISION UART RX bytes={}".format(len(incoming)))
            for packet in parser.feed(incoming):
                if validate_request(packet):
                    print("VISION REQUEST token={} mode={} target={} flags={} action={}".format(
                        packet["token"], packet["mode"], packet["target"],
                        packet["flags"], packet["values"][7]))
                    if packet["flags"] & FLAG_TRACKING and packet["values"][7] == TRACKING_END:
                        if (request is not None and request["flags"] & FLAG_TRACKING and
                                all(packet[key] == request[key]
                                    for key in ("token", "mode", "selector", "target"))):
                            request = None
                            tracking_ring = None
                        continue
                    continuing_layout = request is not None and bool(
                        request["flags"] & FLAG_MATERIAL_LAYOUT)
                    if not packet["flags"] & FLAG_MATERIAL_LAYOUT or (
                            not continuing_layout and not packet["selector"] and
                            packet["values"][7] != LAYOUT_END):
                        layout_memory.reset()
                    tracking_ring = None
                    if packet["flags"] & FLAG_TRACKING and packet["mode"] == MODE_RING and ring_seed:
                        seed_request, seed_center, seed_at = ring_seed
                        if (now_ms - seed_at <= 1800 and
                                all(packet[key] == seed_request[key] for key in ("mode", "selector", "target")) and
                                packet["flags"] & FLAG_RING_NO_DIGIT == seed_request["flags"] & FLAG_RING_NO_DIGIT and
                                packet["values"][:6] == seed_request["values"][:6]):
                            tracking_ring = seed_center
                    ring_seed = None  # A confirmed sample authorizes just the following stream.
                    if packet["flags"] & FLAG_TRACKING and packet["mode"] == MODE_RING:
                        print("VISION TRACK RING token={} reference={}".format(
                            packet["token"], "ready" if tracking_ring is not None else "missing"))
                    request = packet
                    layout_window.reset()
                    if packet["flags"] & FLAG_MATERIAL_LAYOUT and packet["values"][7] == LAYOUT_END:
                        request = None
                        material_status = "layout stopped"
                        continue
                    if packet["mode"] == MODE_RING:
                        ring_label = "ring" if packet["flags"] & FLAG_RING_NO_DIGIT else "ring 2"
                    no_digit = packet["mode"] == MODE_RING and bool(packet["flags"] & FLAG_RING_NO_DIGIT)
                    next_scale = 1 if packet["mode"] == MODE_MATERIAL and packet["target"] == 2 else 2
                    resolution_changed = next_scale != scale
                    if packet["mode"] != light_mode or no_digit != ring_no_digit or resolution_changed:
                        if material_contrast is None and (packet["mode"] == MODE_RING or resolution_changed):
                            material_contrast = cam.constrast()
                        if resolution_changed:
                            err.check_raise(cam.set_resolution(320 * next_scale, 240 * next_scale),
                                            "camera resolution")
                            scale = next_scale
                            print("VISION CAMERA resolution={}x{}".format(320 * scale, 240 * scale))
                        # Unnumbered rings use material colors to select the target.
                        contrast = (settings.RING_CONTRAST
                                    if packet["mode"] == MODE_RING and not no_digit else material_contrast)
                        contrast_readback = cam.constrast(contrast)
                        print("VISION CAMERA mode={} no_digit={} contrast_set={} contrast_readback={}".format(
                            packet["mode"], int(no_digit), contrast, contrast_readback))
                        light_mode = packet["mode"]
                        ring_no_digit = no_digit
                    if packet["flags"] & FLAG_STACK_PREPARE:
                        request = None
                        material_status = "stack ready 320x240"
                        material_detail = ""
                        material_metrics = ""
                        continue
                    request_started = now_ms
                    tracking_last_tx = None
                    tracking_sequence = 0
                    tracking_log_at = None
                    discard_frames = 0 if continuing_layout and packet["flags"] & FLAG_MATERIAL_LAYOUT else 2
                    detection_frames = 0
                    detection_started = None
                    ring_hits = 0
                    ring_detect_ms = 0
                    pickup = packet["mode"] == MODE_MATERIAL and packet["target"] == 0
                    if not packet["flags"] & FLAG_TRACKING:
                        stable = StableWindow(count=10 if pickup else
                                              3 if packet["mode"] == MODE_RING else 5,
                                              tolerance=8 if pickup else 2,
                                              minimum_ms=0 if pickup else
                                              500 if packet["mode"] == MODE_MATERIAL else 180,
                                              trend_tolerance=2 if pickup else None)
                    if packet["mode"] == MODE_MATERIAL:
                        material_status = "material waiting"
                        material_detail = ""
                        material_metrics = ""
            capture_started = int(time.monotonic() * 1000)
            frame = cam.read()
            found = None
            tracking_request = request is not None and bool(request["flags"] & FLAG_TRACKING)
            layout_request = request is not None and bool(request["flags"] & FLAG_MATERIAL_LAYOUT)
            if tracking_request:
                if frame is not None:
                    if discard_frames:
                        discard_frames -= 1
                    else:
                        if request["mode"] == MODE_RING:
                            found = find_ring(frame, request, reference=tracking_ring)
                            tracking_ring = found
                        else:
                            found = find_material(frame, request, scale=scale)
                        # Count every newly detected frame, including rate-limited ones.
                        tracking_sequence = (tracking_sequence + 1) & 0xFFFF
                        now_ms = int(time.monotonic() * 1000)
                        interval_ms = 50 if request["mode"] == MODE_RING else 100
                        if tracking_last_tx is None or now_ms - tracking_last_tx >= interval_ms:
                            values = tracking_values(found, tracking_sequence, now_ms - capture_started)
                            flags = FLAG_TRACKING | (FLAG_VALID if found is not None else 0)
                            response = encode(KIND_RESULT, request["token"], request["mode"],
                                              request["selector"], request["target"], flags, values)
                            if serial.write(response) != len(response):
                                raise RuntimeError("UART short write")
                            tracking_last_tx = now_ms
                            if tracking_log_at is None or now_ms - tracking_log_at >= 1000:
                                print("VISION TRACK TX token={} seq={} valid={} age_ms={}".format(
                                    request["token"], tracking_sequence, found is not None,
                                    values[7]))
                                tracking_log_at = now_ms
            elif layout_request:
                layout = None
                if frame is not None:
                    if discard_frames:
                        discard_frames -= 1
                    else:
                        layout = layout_memory.resolve(find_material_layout(frame, request), request)
                        detection_frames += 1
                        if detection_started is None:
                            detection_started = now_ms
                target_color = request["selector"]
                confirmed = layout_window.update(layout, target_color)
                if confirmed and not target_color and request["values"][7] in (LAYOUT_CONFIRM, LAYOUT_LOCATE):
                    layout_memory.remember(layout)
                values = [0] * 8
                flags = FLAG_MATERIAL_LAYOUT
                if layout is not None:
                    target_slot = (layout["colors"].index(target_color)
                                   if target_color and target_color in layout["colors"] else 0)
                    target_slot = layout.get("reference", target_slot)
                    found = (layout["points"][target_slot]
                             if not target_color or target_color in layout["colors"] else None)
                    values[5:8] = layout["colors"]
                    if layout["inferred"]:
                        flags |= FLAG_COLOR_INFERRED
                    if "reference" in layout:
                        flags |= FLAG_LAYOUT_REFERENCE
                    if found is not None:
                        values[:5] = found[:5]
                        flags |= FLAG_VALID
                        if confirmed and request["values"][7] in (LAYOUT_CONFIRM, LAYOUT_LOCATE):
                            values[:2] = layout_window.center(target_color)
                            flags |= FLAG_STABLE
                    if frame is not None:
                        for slot, point in enumerate(layout.get("visible_points", layout["points"])):
                            if point is not None:
                                label = ("seen" if "visible_points" in layout else
                                         ("bottom", "left_up", "right_up")[slot])
                                frame.draw_cross(point[0] * 2, point[1] * 2, image.COLOR_GREEN, 8, 2)
                                frame.draw_string(point[0] * 2, point[1] * 2,
                                                  "{} c={}".format(label, point[5]), image.COLOR_GREEN)
                now_ms = int(time.monotonic() * 1000)
                elapsed = now_ms - detection_started if detection_started is not None else 0
                fps = (detection_frames - 1) * 1000 / elapsed if elapsed > 0 else 0
                material_status = "layout bottom/left/right={} inferred={} still={}".format(
                    "/".join(str(color) for color in values[5:8]),
                    int(bool(flags & FLAG_COLOR_INFERRED)), int(bool(flags & FLAG_STABLE)))
                material_detail = ("target=(320,410) center=({},{})".format(values[0] * 2, values[1] * 2)
                                   if found is not None else "target=(320,410) bottom missing")
                if target_color:
                    material_detail = ("pick c={} {}=({},{})".format(
                        target_color, "reference" if flags & FLAG_LAYOUT_REFERENCE else "center",
                        values[0] * 2, values[1] * 2) if found is not None else
                        "pick c={} missing".format(target_color))
                material_metrics = "fps={:.1f} elapsed={}ms".format(fps, now_ms - request_started)
                if layout_last_tx is None or now_ms - layout_last_tx >= 100:
                    response = encode(KIND_RESULT, request["token"], request["mode"],
                                      request["selector"], request["target"], flags, values)
                    if serial.write(response) != len(response):
                        raise RuntimeError("UART short write")
                    layout_last_tx = now_ms
            if request is not None and not layout_request and not tracking_request and (frame is not None or
                    (request["mode"] == MODE_MATERIAL and request["target"] == 0)):
                detected = False
                if frame is None:
                    pass  # A missing pickup frame resets the consecutive-frame window.
                elif discard_frames:
                    discard_frames -= 1
                else:
                    if request["mode"] == MODE_RING:
                        detect_started = time.monotonic()
                        found = find_ring(frame, request)
                        ring_detect_ms += int((time.monotonic() - detect_started) * 1000)
                        ring_hits += int(found is not None)
                    else:
                        found = find_material(frame, request, scale=scale)
                    detected = True
                if request["mode"] == MODE_MATERIAL:
                    now_ms = int(time.monotonic() * 1000)
                if detected:
                    detection_frames += 1
                    if detection_started is None:
                        detection_started = now_ms
                confirmed = stable.update(found, now_ms)
                timeout_ms = 1200 if request["mode"] == MODE_MATERIAL else 1000
                values = [0] * 8
                if found is not None:
                    values[:len(found)] = found
                    if confirmed and request["mode"] == MODE_MATERIAL and request["target"] == 0:
                        values[:2] = stable.center()
                if request["mode"] == MODE_MATERIAL:
                    elapsed = now_ms - detection_started if detection_started is not None else 0
                    fps = (detection_frames - 1) * 1000 / elapsed if elapsed > 0 else 0
                    material_metrics = "fps={:.1f} elapsed={}ms".format(
                        fps, now_ms - request_started)
                    if found is None:
                        material_status = "material missing still=0"
                        material_detail = ""
                    else:
                        color = ("unknown", "red", "yellow", "blue", "green", "black",
                                 "light_blue")[found[5]]
                        dx, dy = (values[0] - 160) * scale, (120 - values[1]) * scale
                        material_status = "{} center=({},{}) still={}".format(
                            color, values[0] * scale, values[1] * scale, int(confirmed))
                        material_detail = "dx={} dy={} d={:.1f}px".format(
                            dx, dy, math.hypot(dx, dy))
                if confirmed or now_ms - request_started >= timeout_ms:
                    accepted = confirmed and found is not None
                    flags = FLAG_VALID | FLAG_STABLE if accepted else 0
                    response = encode(KIND_RESULT, request["token"], request["mode"],
                                      request["selector"], request["target"],
                                      flags, values)
                    if serial.write(response) != len(response):
                        raise RuntimeError("UART short write")
                    if accepted and request["mode"] == MODE_RING:
                        ring_seed = (request, found, int(time.monotonic() * 1000))
                    print("VISION RESULT token={} valid={}".format(
                        request["token"], bool(flags)))
                    if request["mode"] == MODE_MATERIAL:
                        print("VISION MATERIAL {} {} {}".format(
                            material_status, material_detail, material_metrics))
                    else:
                        samples = stable.samples
                        span = [max(point[axis] for point in samples) -
                                min(point[axis] for point in samples) if samples else 0
                                for axis in (0, 1)]
                        window_ms = now_ms - stable.started_ms if stable.started_ms is not None else 0
                        print("VISION RING token={} valid={} found={} center=({},{}) "
                              "frames={} hits={} window={} window_ms={} span={}/{}px "
                              "elapsed_ms={} detect_ms={}".format(
                                  request["token"], accepted, found is not None, values[0], values[1],
                                  detection_frames, ring_hits, len(samples), window_ms, span[0], span[1],
                                  int(time.monotonic() * 1000) - request_started, ring_detect_ms))
                    request = None
            if frame is not None:
                if light_mode == MODE_MATERIAL:
                    frame.draw_cross(160 * scale, 120 * scale, image.COLOR_WHITE, 10, 1)
                    for index, line in enumerate((material_status, material_detail,
                                                   material_metrics)):
                        frame.draw_string(2, 2 + index * 18, line, image.COLOR_WHITE)
                if request is not None:
                    x, y, width, height, u, v, _, _ = request["values"]
                    frame.draw_rect(x * scale, y * scale, width * scale, height * scale,
                                    image.COLOR_YELLOW, 1)
                    frame.draw_cross(u * scale, v * scale, image.COLOR_RED, 8, 2)
                if found is not None:
                    frame.draw_cross(int(found[0] * scale), int(found[1] * scale),
                                     image.COLOR_GREEN, 8, 2)
                    if light_mode == MODE_RING:
                        frame.draw_string(2, 2, ring_label, image.COLOR_GREEN)
                screen.show(frame)
    finally:
        illuminator.value(0)
        serial.close()


if __name__ == "__main__":
    main()

"""Three-slot raw-material detection; coordinates use the 320x240 protocol."""
import vision_settings as settings
from vision_protocol import LAYOUT_LOCATE, LAYOUT_TRACK


def find_material_layout(img, request, scale=2):
    roi = tuple(int(value * scale) for value in request["values"][:4])
    # Reference area limits use 640x480 camera pixels, not protocol pixels.
    area_scale = (scale / 2) ** 2
    mask = request["values"][6]
    expected = {color for color in settings.COLOR_THRESHOLDS
                if mask & (1 << (color - 1))}
    recheck = request["selector"] != 0
    if not 1 <= len(expected) <= 3 or (not recheck and len(expected) != 3):
        return None
    candidates = []
    for color in sorted(expected):
        blobs = img.find_blobs([settings.COLOR_THRESHOLDS[color]], roi=roi,
                               area_threshold=50, pixels_threshold=50)
        if not blobs:
            continue
        blob = max(blobs, key=lambda blob: blob.area())
        area = blob.area()
        min_area = 4000 if color == 5 else 1500
        if not min_area * area_scale <= area <= 30000 * area_scale:
            continue
        candidates.append((area, color, blob))
    candidates.sort(key=lambda candidate: candidate[0], reverse=True)
    kept = []
    points = []
    for area, color, blob in candidates:
        x, y, width, height = blob.x(), blob.y(), blob.w(), blob.h()
        # Any positive overlap keeps only the larger color candidate (IoU > 0).
        if any(min(x + width, bx + bw) > max(x, bx) and
               min(y + height, by + bh) > max(y, by) for bx, by, bw, bh in kept):
            continue
        kept.append((x, y, width, height))
        points.append((int(round(blob.cx() / scale)), int(round(blob.cy() / scale)),
                       min(100, int(blob.pixels() * 100 / max(area, 1))),
                       int(round(width / scale)), int(round(height / scale)), color))
    candidates = points
    if not candidates:
        return None
    initial = None
    if not recheck:
        point = max(candidates, key=lambda point: point[1])
        initial = {"points": (point, None, None), "colors": (point[5], 0, 0),
                   "inferred": False, "visible_points": tuple(candidates)}
    if not recheck and (len(candidates) == 1 or request["values"][7] == LAYOUT_TRACK):
        return initial
    # Slot order: bottom, left upper, right upper. A rotating intermediate layout
    # must not be mistaken for the three fixed grasp positions.
    points = [None, None, None]
    bottom = max(candidates, key=lambda point: point[1])
    for point in candidates:
        if point[1] >= 120:
            slot = 0
            if point is not bottom:
                return initial if request["values"][7] == LAYOUT_LOCATE else None
        elif point[0] < 160:
            slot = 1
        elif point[0] > 160:
            slot = 2
        else:
            return initial if request["values"][7] == LAYOUT_LOCATE else None
        if points[slot] is not None:
            return initial if request["values"][7] == LAYOUT_LOCATE else None
        points[slot] = point
    if not recheck and request["values"][7] == LAYOUT_LOCATE and points[0] is None:
        return initial
    colors = [point[5] if point else 0 for point in points]
    inferred = not recheck and len(candidates) == 2
    if inferred:
        colors[colors.index(0)] = next(iter(expected - set(colors)))
    result = {"points": tuple(points), "colors": tuple(colors), "inferred": inferred}
    if not recheck and request["values"][7] == LAYOUT_LOCATE:
        result["visible_points"] = tuple(candidates)
    return result


class LayoutMemory:
    def __init__(self):
        self.reset()

    def reset(self):
        self.colors = None

    def remember(self, layout):
        if (layout is not None and all(layout["colors"]) and
                len(set(layout["colors"])) == 3 and layout["points"][0] is not None):
            self.colors = layout["colors"]

    def resolve(self, layout, request):
        if not request["selector"] or self.colors is None or layout is None:
            return layout
        expected = {color for color in range(1, 7) if request["values"][6] & (1 << (color - 1))}
        if not expected <= set(self.colors):
            return None
        matches = []
        colors = self.colors
        for _ in range(3):
            candidate = tuple(color if color in expected else 0 for color in colors)
            if all(point is None or candidate[slot] == point[5]
                   for slot, point in enumerate(layout["points"])):
                matches.append(candidate)
            # Image counterclockwise: bottom -> right upper -> left upper -> bottom.
            colors = (colors[1], colors[2], colors[0])
        if len(matches) != 1:
            return None
        colors = matches[0]
        inferred = any(color and layout["points"][slot] is None for slot, color in enumerate(colors))
        result = dict(layout, colors=colors, inferred=inferred)
        if result["points"][colors.index(request["selector"])] is None:
            # Return a real visible reference, never a fabricated target center.
            result["reference"] = next(slot for slot, point in enumerate(result["points"]) if point)
        return result


class LayoutWindow:
    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = []
        self.signature = None
        self.moving = False

    def update(self, layout, target_color=0):
        target_slot = (layout["colors"].index(target_color)
                       if layout is not None and target_color and target_color in layout["colors"] else 0)
        if layout is not None:
            target_slot = layout.get("reference", target_slot)
        if (layout is None or (target_color and target_color not in layout["colors"]) or
                layout["points"][target_slot] is None):
            self.reset()
            return False
        signature = (layout["colors"], tuple(point is not None for point in layout["points"]), target_slot)
        if signature != self.signature:
            self.reset()
            self.signature = signature
        self.samples.append(layout["points"])
        self.samples = self.samples[-6:]
        if len(self.samples) < 6:
            return False
        still = any(
            all(max(row[slot][axis] for row in self.samples) -
                min(row[slot][axis] for row in self.samples) <= 8 and
                abs(sorted(row[slot][axis] for row in self.samples[:3])[1] -
                    sorted(row[slot][axis] for row in self.samples[-3:])[1]) <= 2
                for axis in (0, 1))
            for slot, point in enumerate(layout["points"]) if point is not None)
        self.moving = not still
        return still

    def center(self, target_color=0):
        slot = self.signature[2]
        return [int(round((sorted(row[slot][axis] for row in self.samples)[2] +
                           sorted(row[slot][axis] for row in self.samples)[3]) / 2))
                for axis in (0, 1)]

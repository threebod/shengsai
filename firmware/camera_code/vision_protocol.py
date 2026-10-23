"""Vehicle vision protocol: fixed 30-byte little-endian CRC frames."""
import struct

FRAME_SIZE = 30
FORMAT = "<2sBBIBBBB8hH"
KIND_REQUEST = 1
KIND_RESULT = 2
MODE_MATERIAL = 1
MODE_RING = 2
FLAG_VALID = 1
FLAG_STABLE = 2
FLAG_RING_NO_DIGIT = 4  # Request only; selector carries the fallback material color.
FLAG_MATERIAL_LAYOUT = 8
FLAG_COLOR_INFERRED = 16  # Result only; one slot color was inferred.
FLAG_STACK_PREPARE = 32  # Request only; configure stacking without detecting or replying.
FLAG_LAYOUT_REFERENCE = 64  # Result only; center belongs to a visible rotation reference.
FLAG_TRACKING = 128  # Continuous raw results for chassis alignment.
TRACKING_START = 1
TRACKING_END = -1
LAYOUT_TRACK = 0
LAYOUT_CONFIRM = 1
LAYOUT_END = 2
LAYOUT_LOCATE = 3  # First parked alignment, before three-slot color confirmation.


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode(kind, token, mode, selector, target, flags, values):
    if len(values) != 8:
        raise ValueError("values must contain eight int16 fields")
    frame = struct.pack(FORMAT, b"\xA5\x5A", 1, kind, token, mode,
                        selector, target, flags, *values, 0)
    return frame[:28] + struct.pack("<H", crc16(frame[:28]))


def tracking_values(found, sequence, age_ms):
    values = [0] * 8
    if found is not None:
        values[:len(found)] = found
    sequence &= 0xFFFF
    values[6] = sequence if sequence < 0x8000 else sequence - 0x10000
    values[7] = max(0, min(32767, int(age_ms)))
    return values


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"\xA5\x5A":
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            expected = struct.unpack_from("<H", frame, 28)[0]
            if frame[2] != 1 or crc16(frame[:28]) != expected:
                del self.buffer[0]
                continue
            row = struct.unpack(FORMAT, frame)
            packets.append({"kind": row[2], "token": row[3], "mode": row[4],
                            "selector": row[5], "target": row[6],
                            "flags": row[7], "values": list(row[8:16])})
            del self.buffer[:FRAME_SIZE]
        return packets


class StableWindow:
    def __init__(self, count=5, tolerance=2, minimum_ms=180, trend_tolerance=None):
        self.count = count
        self.tolerance = tolerance
        self.minimum_ms = minimum_ms
        self.trend_tolerance = trend_tolerance
        self.reset()

    def reset(self):
        self.samples = []
        self.started_ms = None

    def update(self, point, now_ms):
        if point is None:
            self.reset()
            return False
        next_samples = self.samples + [point]
        if self.samples and any(
                max(sample[i] for sample in next_samples) -
                min(sample[i] for sample in next_samples) > self.tolerance
                for i in (0, 1)):
            self.reset()
        if self.samples and len(point) > 5 and point[5] != self.samples[-1][5]:
            self.reset()
        if not self.samples:
            self.started_ms = now_ms
        self.samples.append(point)
        if len(self.samples) < self.count or now_ms - self.started_ms < self.minimum_ms:
            return False
        # Pickup compares the first and last five-frame center medians.
        if self.trend_tolerance is not None and any(
                abs(sorted(sample[axis] for sample in self.samples[:5])[2] -
                    sorted(sample[axis] for sample in self.samples[-5:])[2]) >
                self.trend_tolerance for axis in (0, 1)):
            self.reset()
            self.samples.append(point)
            self.started_ms = now_ms
            return False
        return True

    def center(self):
        centers = []
        for axis in (0, 1):
            values = sorted(sample[axis] for sample in self.samples)
            middle = len(values) // 2
            centers.append(int(round((values[middle] + values[~middle]) / 2)))
        return centers


def validate_request(packet):
    if packet["kind"] != KIND_REQUEST or packet["mode"] not in (MODE_MATERIAL, MODE_RING):
        return False
    if packet["mode"] == MODE_MATERIAL and not 0 <= packet["selector"] <= 6:
        return False
    if packet["mode"] == MODE_MATERIAL and not 0 <= packet["values"][6] <= 63:
        return False
    tracking = bool(packet["flags"] & FLAG_TRACKING)
    if tracking and (packet["flags"] & ~(FLAG_TRACKING | FLAG_RING_NO_DIGIT) or
            packet["values"][7] not in (TRACKING_START, TRACKING_END) or
            (packet["mode"] == MODE_MATERIAL and
             (packet["target"] != 2 or packet["flags"] != FLAG_TRACKING))):
        return False
    if packet["flags"] & FLAG_STACK_PREPARE and (
            packet["mode"] != MODE_MATERIAL or packet["target"] != 2 or
            packet["flags"] != FLAG_STACK_PREPARE):
        return False
    if packet["flags"] & FLAG_MATERIAL_LAYOUT:
        mask = packet["values"][6]
        selector = packet["selector"]
        colors_valid = (bin(mask).count("1") == 3 if selector == 0 else
                        1 <= selector <= 6 and 1 <= bin(mask).count("1") <= 3 and
                        bool(mask & (1 << (selector - 1))))
        if (packet["mode"] != MODE_MATERIAL or packet["target"] != 0 or
                not colors_valid or
                (selector != 0 and packet["values"][7] == LAYOUT_LOCATE) or
                packet["values"][7] not in (LAYOUT_TRACK, LAYOUT_CONFIRM, LAYOUT_END, LAYOUT_LOCATE)):
            return False
    if packet["mode"] == MODE_RING and not 1 <= packet["target"] <= 3:
        return False
    if packet["mode"] == MODE_RING and packet["flags"] & FLAG_RING_NO_DIGIT and (
            packet["target"] != 2 or not 1 <= packet["selector"] <= 6):
        return False
    if tracking and packet["values"][7] == TRACKING_END:
        return True
    x, y, width, height, u, v, _, _ = packet["values"]
    return (0 <= x < 320 and 0 <= y < 240 and width > 0 and height > 0 and
            x + width <= 320 and y + height <= 240 and
            x <= u < x + width and y <= v < y + height)

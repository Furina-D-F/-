import struct


SOF = b"\xAA\x55"
VERSION = 1
MAX_PAYLOAD = 128
HEADER_FORMAT = "<2sBBHBBB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD + 2

FRAME_COMMAND = 0x01
FRAME_RESPONSE = 0x02
FRAME_STATUS = 0x03

CMD_MOTION = 0x01
CMD_CONFIG = 0x02
CMD_STATUS = 0x03

STATUS_OK = 0x00
STATUS_BAD_LENGTH = 0x01
STATUS_BAD_COMMAND = 0x02
STATUS_BAD_CRC = 0x03
STATUS_TIMEOUT = 0x04
STATUS_DUPLICATE = 0x05
STATUS_OVERFLOW = 0x06


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def encode(frame_type: int, sequence: int, command: int,
           response_code: int = STATUS_OK, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 128 bytes")
    header = struct.pack(
        HEADER_FORMAT, SOF, VERSION, frame_type, len(payload),
        sequence & 0xFF, command & 0xFF, response_code & 0xFF
    )
    body = header + payload
    return body + struct.pack("<H", crc16(body))


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_sequence = None

    def feed(self, data: bytes):
        frames = []
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < HEADER_SIZE:
                break
            _, version, frame_type, payload_length, sequence, command, response_code = struct.unpack(
                HEADER_FORMAT, self.buffer[:HEADER_SIZE]
            )
            if version != VERSION or payload_length > MAX_PAYLOAD:
                del self.buffer[:2]
                continue
            frame_length = HEADER_SIZE + payload_length + 2
            if len(self.buffer) < frame_length:
                break
            raw = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]
            expected_crc = struct.unpack("<H", raw[-2:])[0]
            if crc16(raw[:-2]) != expected_crc:
                continue
            payload = raw[HEADER_SIZE:-2]
            duplicate = self.last_sequence == sequence
            self.last_sequence = sequence
            frames.append({
                "type": frame_type,
                "sequence": sequence,
                "command": command,
                "response_code": response_code,
                "payload": payload,
                "duplicate": duplicate,
            })
        return frames
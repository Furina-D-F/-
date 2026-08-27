import socket
import struct

from protocol import (
    CMD_STATUS,
    FRAME_COMMAND,
    FRAME_RESPONSE,
    FrameParser,
    STATUS_OK,
    encode,
)


def main():
    host, firmware = socket.socketpair()
    try:
        request = encode(FRAME_COMMAND, 1, CMD_STATUS)
        host.sendall(request)

        received = bytearray()
        while len(received) < len(request):
            received.extend(firmware.recv(64))

        request_parser = FrameParser()
        request_frame = request_parser.feed(bytes(received))[0]
        response = encode(
            FRAME_RESPONSE,
            request_frame["sequence"],
            request_frame["command"],
            STATUS_OK,
        )
        firmware.sendall(response)

        response_parser = FrameParser()
        response_frame = response_parser.feed(host.recv(64))[0]
        assert response_frame["type"] == FRAME_RESPONSE
        assert response_frame["sequence"] == 1
        assert response_frame["response_code"] == STATUS_OK
        print("bidirectional protocol link: PASS")
    finally:
        host.close()
        firmware.close()


if __name__ == "__main__":
    main()
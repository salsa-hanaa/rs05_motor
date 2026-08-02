import serial
import time


class SerialTransport:
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.2):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._init_handshake()

    def _init_handshake(self):
        """Kirim 'AT+AT' buat inisialisasi adapter USB-CAN (wajib pertama kali)."""
        self.ser.write(b"AT+AT\r\n")
        time.sleep(0.05)
        return self.ser.read(64)

    def send_frame(self, frame: bytes, read_reply: bool = True) -> bytes:
        """Kirim bytes perintah lewat serial dan baca balasannya jika ada.

        Always discard whatever is sitting in the input buffer BEFORE sending
        a new frame. This matters because some frames (e.g. MOTION_CONTROL
        sent with read_reply=False) can still trigger a reply from the motor
        that is never consumed. If left there, that stale reply gets read by
        the NEXT send_frame() call (e.g. read_joints) and gets parsed as if
        it were that call's own response, producing a garbage/misaligned
        position reading.
        """
        self.ser.reset_input_buffer()
        self.ser.write(frame)

        if read_reply:
            return self.ser.read(64)
        return b""

    def close(self):
        if self.ser.is_open:
            self.ser.close()
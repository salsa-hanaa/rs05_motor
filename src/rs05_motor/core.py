import logging
from .communicate import SerialTransport
from .conversion import (
    float_to_uint16, uint16_to_float,
    POS_RANGE, VEL_RANGE, KP_RANGE, KD_RANGE, TORQUE_RANGE
)
from .bit_convert import build_at_frame, COMM_TYPE_ENABLE, COMM_TYPE_STOP, COMM_TYPE_MOTION, DEFAULT_HOST_ID
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class RS05Motor:
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.2):
        logging.info(f"Menghubungkan ke port {port} dengan baudrate {baudrate}...")
        self.transport = SerialTransport(port, baudrate, timeout)
        self.saved_motions = {}
        logging.info("Inisialisasi handshake port selesai.")

    def _execute_and_log(self, action_name: str, frame: bytes,
                          read_reply: bool = True, verbose: bool = True) -> bytes:
        # if verbose:
            # print(f"\n[{action_name}]")
            # print(f"  TxHex: \"{frame.hex()}\"")

        rx_bytes = self.transport.send_frame(frame, read_reply=read_reply)

        # if verbose:
        #     print(f"  RxHex: \"{rx_bytes.hex() if rx_bytes else 'No Response/Timeout'}\"")
        return rx_bytes

    def enable(self, motor_id: int):
        payload = bytes(8)
        frame = build_at_frame(COMM_TYPE_ENABLE, DEFAULT_HOST_ID, motor_id, payload)
        logging.info(f"Mengirim perintah ENABLE ke Motor ID: {motor_id}")
        return self._execute_and_log("ENABLE_MOTOR", frame)

    def stop(self, motor_id: int):
        payload = bytes(8)
        frame = build_at_frame(COMM_TYPE_STOP, DEFAULT_HOST_ID, motor_id, payload)
        logging.info(f"Mengirim perintah STOP ke Motor ID: {motor_id}")
        return self._execute_and_log("STOP_MOTOR", frame)

    def move_gripper(self, motor_id: int, angle_rad: float, kp: float, kd: float,
                     velocity: float = 0.0, torque: float = 0.0):
        logging.debug(
            "Kirim Perintah Gerak -> ID: %d | Angle: %.2f | Kp: %.1f | Kd: %.1f",
            motor_id, angle_rad, kp, kd
        )

        pos_raw = float_to_uint16(angle_rad, *POS_RANGE)
        vel_raw = float_to_uint16(velocity, *VEL_RANGE)
        kp_raw  = float_to_uint16(kp, *KP_RANGE)
        kd_raw  = float_to_uint16(kd, *KD_RANGE)
        torque_raw = float_to_uint16(torque, *TORQUE_RANGE)

        payload = (
            pos_raw.to_bytes(2, "big") +
            vel_raw.to_bytes(2, "big") +
            kp_raw.to_bytes(2, "big") +
            kd_raw.to_bytes(2, "big")
        )

        frame = build_at_frame(COMM_TYPE_MOTION, torque_raw, motor_id, payload)
        return self._execute_and_log("MOTION_CONTROL", frame, read_reply=False, verbose=False)


    def read_joints(self, motor_id: int) -> float:
            payload = bytes(8)
            frame = build_at_frame(COMM_TYPE_ENABLE, DEFAULT_HOST_ID, motor_id, payload)
            rx_bytes = self._execute_and_log("READ_JOINTS_TRIGGER", frame)

            if not rx_bytes or len(rx_bytes) < 17:
                # logging.warning("Respons tidak lengkap/timeout saat read_joints, kembalikan 0.0 rad.")
                return 0.0

            data_field = rx_bytes[7:15]
            pos_raw = int.from_bytes(data_field[0:2], "big")
            angle_rad = uint16_to_float(pos_raw, *POS_RANGE)
            return angle_rad

    def check_joints(self, motor_id: int) -> float:
        """Baca dan cetak posisi motor saat ini secara instan (untuk monitoring)."""
        angle_rad = self.read_joints(motor_id)
        # print(f"[CHECK_JOINTS] Motor ID {motor_id} -> Posisi saat ini: {angle_rad:.4f} rad")
        return angle_rad

    def save_motion(self, name: str, angle_rad: float):
        """Simpan posisi (rad) ke memori gerakan dengan key nama gerakan."""
        self.saved_motions[name] = angle_rad
        # logging.info(f"Gerakan '{name}' disimpan pada posisi {angle_rad:.4f} rad")

    def play_motion(self, motor_id: int, name: str, kp: float, kd: float):
        """Mainkan ulang gerakan yang sudah disimpan sebelumnya via save_motion."""
        if name not in self.saved_motions:
            logging.error(f"Gerakan '{name}' tidak ditemukan di saved_motions.")
            return None

        angle_rad = self.saved_motions[name]
        logging.info(f"Memutar ulang gerakan '{name}' -> target {angle_rad:.4f} rad")
        return self.move_gripper(motor_id, angle_rad=angle_rad, kp=kp, kd=kd)

    def close(self):
        logging.info("Menutup koneksi serial.")
        self.transport.close()
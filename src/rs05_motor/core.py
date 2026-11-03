import logging
import struct
import time
from typing import Optional

from .communicate import SerialTransport
from .conversion import (
    float_to_uint16, uint16_to_float,
    POS_RANGE, VEL_RANGE, KP_RANGE, KD_RANGE, TORQUE_RANGE
)
from .bit_convert import (
    build_at_frame, decode_can_id,
    COMM_TYPE_ENABLE, COMM_TYPE_STOP, COMM_TYPE_MOTION, COMM_TYPE_SET_ZERO,
    COMM_TYPE_FEEDBACK, COMM_TYPE_READ_PARAM, COMM_TYPE_PARAM_WR, COMM_TYPE_SAVE,
    PARAM_MECH_POS, PARAM_ZERO_STA,
    DEFAULT_HOST_ID
)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

FRAME_LEN = 17  # 'AT'(2) + CAN_ID(4) + DLC(1) + data(8) + '\r\n'(2)


class RS05Motor:
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.2):
        logging.info(f"Menghubungkan ke port {port} dengan baudrate {baudrate}...")
        self.transport = SerialTransport(port, baudrate, timeout)
        self.saved_motions = {}
        logging.info("Inisialisasi handshake port selesai.")

    # ------------------------------------------------------------------ helpers
    def _execute_and_log(self, action_name: str, frame: bytes,
                         read_reply: bool = True, verbose: bool = True) -> bytes:
        return self.transport.send_frame(frame, read_reply=read_reply)

    @staticmethod
    def _extract_frame(rx_bytes: bytes) -> Optional[bytes]:
        """Ambil satu frame 17-byte yang diawali 'AT' dari balasan mentah."""
        if not rx_bytes:
            return None
        start = rx_bytes.find(b"AT")
        if start < 0:
            return None
        frame = rx_bytes[start:start + FRAME_LEN]
        return frame if len(frame) == FRAME_LEN else None

    @staticmethod
    def _reply_matches(frame: bytes, motor_id: int, expected_type: int) -> bool:
        """Cek tipe komunikasi dan Motor ID pada balasan.

        Pada balasan type 2, data_area = motor_id (bit 8-15) + fault flags (bit 16-21)
        + mode (bit 22-23). Jadi HANYA byte rendah yang dibandingkan dengan motor_id.
        """
        comm_type, data_area, _ = decode_can_id(frame[2:6])
        return comm_type == expected_type and (data_area & 0xFF) == motor_id

    # ------------------------------------------------------------ basic commands
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

    # ------------------------------------------------------- parameter read/write
    def read_param(self, motor_id: int, index: int) -> Optional[float]:
        """Type 17: baca satu parameter float. Tidak mengubah state motor."""
        payload = index.to_bytes(2, "little") + bytes(6)
        frame = build_at_frame(COMM_TYPE_READ_PARAM, DEFAULT_HOST_ID, motor_id, payload)
        rx_bytes = self._execute_and_log("READ_PARAM", frame)

        rx = self._extract_frame(rx_bytes)
        if rx is None:
            logging.warning(f"read_param 0x{index:04X}: tidak ada/kurang respons dari Motor ID {motor_id}.")
            return None

        if not self._reply_matches(rx, motor_id, COMM_TYPE_READ_PARAM):
            logging.warning(f"read_param 0x{index:04X}: balasan bukan type 17 dari Motor ID {motor_id}.")
            return None

        data = rx[7:15]
        if int.from_bytes(data[0:2], "little") != index:
            logging.warning(f"read_param 0x{index:04X}: index pada balasan tidak cocok.")
            return None

        return struct.unpack("<f", data[4:8])[0]  # little-endian IEEE-754

    def write_param_u8(self, motor_id: int, index: int, value: int) -> bool:
        """Type 18: tulis parameter uint8 (hilang saat power off kecuali di-save dengan type 22)."""
        payload = index.to_bytes(2, "little") + bytes(2) + bytes([value & 0xFF]) + bytes(3)
        frame = build_at_frame(COMM_TYPE_PARAM_WR, DEFAULT_HOST_ID, motor_id, payload)
        rx = self._extract_frame(self._execute_and_log("WRITE_PARAM_U8", frame))
        return rx is not None and self._reply_matches(rx, motor_id, COMM_TYPE_FEEDBACK)

    def save_params(self, motor_id: int) -> bool:
        """Type 22: simpan parameter ke flash agar bertahan setelah power off."""
        payload = bytes([1, 2, 3, 4, 5, 6, 7, 8])
        frame = build_at_frame(COMM_TYPE_SAVE, DEFAULT_HOST_ID, motor_id, payload)
        logging.info(f"Menyimpan parameter (type 22) ke Motor ID: {motor_id}")
        rx = self._extract_frame(self._execute_and_log("SAVE_PARAMS", frame))
        ok = rx is not None and self._reply_matches(rx, motor_id, COMM_TYPE_FEEDBACK)
        if not ok:
            logging.warning(f"Tidak ada konfirmasi save dari Motor ID {motor_id}.")
        return ok

    def set_zero_sta(self, motor_id: int, value: int = 1, save: bool = True) -> bool:
        """zero_sta: 0 -> range posisi saat power-on 0..2pi, 1 -> -pi..pi.

        Dengan 0, posisi sedikit di bawah nol (misal -0.02 rad) akan terbaca ~6.26 rad
        setelah power cycle, seolah-olah zero bergeser. Panggil sekali saat setup.
        """
        ok = self.write_param_u8(motor_id, PARAM_ZERO_STA, value)
        if not ok:
            logging.warning(f"Gagal menulis zero_sta ke Motor ID {motor_id}.")
            return False
        time.sleep(0.05)
        return self.save_params(motor_id) if save else True

    # ------------------------------------------------------------------ zeroing
    def set_zero(self, motor_id: int, save: bool = True) -> bool:
        """Set posisi joint saat ini sebagai mechanical zero (joint -> 0).

        save=True juga mengirim type 22 supaya zero bertahan setelah power off.
        """
        payload = bytes([1, 0, 0, 0, 0, 0, 0, 0])
        frame = build_at_frame(COMM_TYPE_SET_ZERO, DEFAULT_HOST_ID, motor_id, payload)
        logging.info(f"Mengirim perintah SET ZERO (mechanical zero) ke Motor ID: {motor_id}")
        rx = self._extract_frame(self._execute_and_log("SET_ZERO_POSITION", frame))

        if rx is None:
            logging.warning(f"Tidak ada respons saat set_zero Motor ID {motor_id}.")
            return False

        if not self._reply_matches(rx, motor_id, COMM_TYPE_FEEDBACK):
            logging.warning(f"Respons set_zero Motor ID {motor_id} tidak sesuai (gagal diverifikasi).")
            return False

        logging.info(f"Set the mechanical zero position successfully! Motor ID: {motor_id}")
        if save:
            time.sleep(0.05)
            self.save_params(motor_id)
        return True

    def set_zero_multi(self, motor_ids: list, delay: float = 0.2, save: bool = True) -> dict:
        """Set mechanical zero untuk beberapa Motor ID secara bergilir lewat satu bus CAN yang sama."""
        hasil = {}
        for motor_id in motor_ids:
            logging.info(f"--- Proses set_zero Motor ID {motor_id} ---")
            self.stop(motor_id)
            time.sleep(delay)
            hasil[motor_id] = self.set_zero(motor_id, save=save)
            time.sleep(delay)
        return hasil

    # ------------------------------------------------------------------- motion
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

    def move_multi(self, commands: list):
        """Gerakkan beberapa motor 'bersamaan' dengan menggabungkan semua frame jadi
        satu write() serial, bukan loop kirim-satu-satu (yang punya jeda antar motor).
        commands: list of dict {motor_id, angle_rad, kp, kd, velocity=0.0, torque=0.0}
        """
        combined = bytearray()
        for cmd in commands:
            pos_raw = float_to_uint16(cmd["angle_rad"], *POS_RANGE)
            vel_raw = float_to_uint16(cmd.get("velocity", 0.0), *VEL_RANGE)
            kp_raw  = float_to_uint16(cmd["kp"], *KP_RANGE)
            kd_raw  = float_to_uint16(cmd["kd"], *KD_RANGE)
            torque_raw = float_to_uint16(cmd.get("torque", 0.0), *TORQUE_RANGE)

            payload = (
                pos_raw.to_bytes(2, "big") +
                vel_raw.to_bytes(2, "big") +
                kp_raw.to_bytes(2, "big") +
                kd_raw.to_bytes(2, "big")
            )
            combined += build_at_frame(COMM_TYPE_MOTION, torque_raw, cmd["motor_id"], payload)

        motor_ids = [cmd["motor_id"] for cmd in commands]
        logging.info(f"Mengirim MOTION_CONTROL serentak (1x write) ke Motor ID: {motor_ids}")
        return self._execute_and_log("MOTION_CONTROL_MULTI", bytes(combined), read_reply=False, verbose=False)

    # ------------------------------------------------------------------ reading
    def read_joints(self, motor_id: int) -> Optional[float]:
        """Baca posisi load-end (mechPos, rad) lewat type 17.

        Tidak mengirim ENABLE, jadi tidak mengubah state motor. Mengembalikan None
        kalau gagal, supaya caller bisa membedakan 'gagal baca' dari 'posisi 0.0'.
        """
        return self.read_param(motor_id, PARAM_MECH_POS)

    def check_joints(self, motor_id: int):
        """Baca dan cetak posisi motor saat ini secara instan (untuk monitoring)."""
        angle_rad = self.read_joints(motor_id)
        if angle_rad is None:
            print(f"[CHECK_JOINTS] Motor ID {motor_id} -> Gagal membaca posisi.")
        else:
            print(f"[CHECK_JOINTS] Motor ID {motor_id} -> Posisi saat ini: {angle_rad:.4f} rad")
        return angle_rad

    # ------------------------------------------------------------ saved motions
    def save_motion(self, name: str, angle_rad: float):
        """Simpan posisi (rad) ke memori gerakan dengan key nama gerakan."""
        self.saved_motions[name] = angle_rad

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
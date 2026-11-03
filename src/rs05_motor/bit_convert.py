COMM_TYPE_GET_ID       = 0
COMM_TYPE_MOTION       = 1
COMM_TYPE_FEEDBACK     = 2    
COMM_TYPE_ENABLE       = 3
COMM_TYPE_STOP         = 4
COMM_TYPE_SET_ZERO     = 6
COMM_TYPE_READ_PARAM   = 17   
COMM_TYPE_PARAM_WR     = 18   
COMM_TYPE_SAVE         = 22   
COMM_TYPE_ACTIVE_REPORT = 24  


PARAM_RUN_MODE = 0x7005
PARAM_MECH_POS = 0x7019   # load-end angle, rad (float, read only)
PARAM_MECH_VEL = 0x701B   # load speed, rad/s   (float, read only)
PARAM_VBUS     = 0x701C   # bus voltage, V      (float, read only)
PARAM_ZERO_STA = 0x7029   # 0 -> power-on range 0..2pi, 1 -> -pi..pi (uint8)


DEFAULT_HOST_ID = 0x00FD
CAN_FRAME_FLAGS = 0b100


def build_can_id(comm_type: int, data_area: int, motor_id: int) -> bytes:
    """Susun 4-byte CAN_ID dari comm_type(5bit) + data_area(16bit) + motor_id(8bit)."""
    id29 = ((comm_type & 0x1F) << 24) | ((data_area & 0xFFFF) << 8) | (motor_id & 0xFF)
    raw32 = (id29 << 3) | CAN_FRAME_FLAGS
    return raw32.to_bytes(4, "big")


def decode_can_id(can_id_bytes: bytes):
    """Kebalikan build_can_id - buat baca frame Rx dari motor."""
    id29 = int.from_bytes(can_id_bytes, "big") >> 3
    comm_type = (id29 >> 24) & 0x1F
    data_area = (id29 >> 8) & 0xFFFF
    motor_id  = id29 & 0xFF
    return comm_type, data_area, motor_id


def build_at_frame(comm_type: int, data_area: int, motor_id: int, payload: bytes) -> bytes:
    """Bikin satu frame lengkap siap-kirim (bytes ASCII, sudah termasuk AT...\r\n)."""
    can_id = build_can_id(comm_type, data_area, motor_id)
    dlc = bytes([len(payload)])
    frame_bytes = can_id + dlc + payload
    ascii_hex = "4154" + frame_bytes.hex() + "0d0a"  # "AT" + frame + \r\n
    return bytes.fromhex(ascii_hex)
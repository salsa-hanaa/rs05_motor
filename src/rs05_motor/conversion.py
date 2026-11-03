import math

POS_RANGE    = (-4 * math.pi, 4 * math.pi)   
VEL_RANGE    = (-50.0, 50.0)                 
KP_RANGE     = (0.0, 500.0)
KD_RANGE     = (0.0, 5.0)
TORQUE_RANGE = (-6.0, 6.0)                    


def float_to_uint16(value: float, min_val: float, max_val: float) -> int:
    """Ubah nilai float (misal rad, Nm, dst) jadi uint16 mentah yang dikirim ke motor."""
    value = max(min_val, min(max_val, value))
    raw = int((value - min_val) / (max_val - min_val) * 65535)
    return max(0, min(65535, raw))


def uint16_to_float(raw: int, min_val: float, max_val: float) -> float:
    """Kebalikan dari float_to_uint16 - dipakai buat decode data feedback dari motor."""
    return raw / 65535 * (max_val - min_val) + min_val
import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor import RS05Motor

if __name__ == "__main__":
    PORT = "/dev/ttyUSB0"
    MOTOR_IDS = [11, 12]

    print("Menginisialisasi Motor...")
    motor = RS05Motor(PORT)

    try:
        for motor_id in MOTOR_IDS:
            motor.enable(motor_id)
        time.sleep(0.2)

        print(f"Menggerakkan Motor ID {MOTOR_IDS} bersamaan (1x write, tanpa jeda antar motor)...")
        motor.move_multi([
            {"motor_id": MOTOR_IDS[0], "angle_rad": 1.5, "kp": 50.0, "kd": 1.0},
            {"motor_id": MOTOR_IDS[1], "angle_rad": -1.5, "kp": 50.0, "kd": 1.0},
        ])
        time.sleep(1.5)

        print("Mengembalikan kedua motor ke posisi 0.0 rad, tetap bersamaan...")
        motor.move_multi([
            {"motor_id": MOTOR_IDS[0], "angle_rad": 0.0, "kp": 50.0, "kd": 1.0},
            {"motor_id": MOTOR_IDS[1], "angle_rad": 0.0, "kp": 50.0, "kd": 1.0},
        ])
        time.sleep(1.5)

        for motor_id in MOTOR_IDS:
            motor.stop(motor_id)

    finally:
        motor.close()
        print("Demo Selesai.")

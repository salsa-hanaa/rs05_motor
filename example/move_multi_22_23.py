import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor import RS05Motor

if __name__ == "__main__":
    PORT = "/dev/ttyUSB1"

    print("Menginisialisasi Motor...")
    motor = RS05Motor(PORT)

    try:
        print("Mengaktifkan Motor ID 22 dan 23...")
        motor.enable(22)
        motor.enable(23)
        time.sleep(0.2)

        print("Menggerakkan Motor ID 22 -> 1.5 rad dan Motor ID 23 -> -1.5 rad bersamaan...")
        motor.move_multi([
            {"motor_id": 22, "angle_rad": 1.5, "kp": 2.0, "kd": 1.0},
            {"motor_id": 23, "angle_rad": -1.5, "kp": 2.0, "kd": 1.0},
        ])
        time.sleep(1.5)

        print("Menghentikan Motor ID 22 dan 23...")
        motor.stop(22)
        motor.stop(23)

    finally:
        motor.close()
        print("Demo Selesai.")

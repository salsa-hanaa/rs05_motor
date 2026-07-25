import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor import RS05Motor

if __name__ == "__main__":
    PORT = "/dev/ttyUSB1"
    MOTOR_ID = 23

    print("Menginisialisasi Motor...")
    motor = RS05Motor(PORT)

    try:
        print(f"Menghentikan Motor ID {MOTOR_ID} sebelum set zero...")
        motor.stop(MOTOR_ID)
        time.sleep(0.2)

        print(f"Set mechanical zero untuk Motor ID {MOTOR_ID}...")
        berhasil = motor.set_zero(MOTOR_ID)

        if berhasil:
            print("Set the mechanical zero position successfully!")
        else:
            print("Set zero GAGAL, cek koneksi serial / CAN ID motor.")

    finally:
        motor.close()
        print("Demo Selesai.")

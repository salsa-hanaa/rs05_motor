import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor import RS05Motor

if __name__ == "__main__":
    PORT = "/dev/ttyUSB0"
    MOTOR_ID = 23

    print("Menginisialisasi Motor...")
    motor = RS05Motor(PORT)

    try:
        print("Mengaktifkan Motor (Enable)...")
        motor.enable(MOTOR_ID)
        time.sleep(0.2)

        print("Menggerakkan motor ke posisi 3.0 rad...")
        motor.move_gripper(MOTOR_ID, angle_rad=3.0, kp=50.0, kd=1.0)
        time.sleep(1.5)

        print("Membaca posisi aktual (read_joints/check_joints)...")
        posisi_terekam = motor.check_position(MOTOR_ID)

        motor.save_motion("gerak pertama", posisi_terekam)

        print("Mengembalikan motor ke posisi 0.0 rad...")
        motor.move_gripper(MOTOR_ID, angle_rad=0.0, kp=50.0, kd=1.0)
        time.sleep(1.5)

        print("Play motion 'gerak pertama'...")
        motor.play_motion(MOTOR_ID, "gerak pertama", kp=50.0, kd=1.0)
        time.sleep(1.5)

        print("Menghentikan Motor (Stop)...")
        motor.stop(MOTOR_ID)

    finally:
        motor.close()
        print("Demo Selesai.")
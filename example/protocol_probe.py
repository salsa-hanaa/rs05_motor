import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor.bit_convert import build_at_frame, decode_can_id, DEFAULT_HOST_ID
from rs05_motor.communicate import SerialTransport

KNOWN_COMM_TYPES = {
    0: "GET_ID",
    1: "MOTION (MIT control)",
    3: "ENABLE",
    4: "STOP",
    6: "SET_ZERO (sudah ketemu dari capture set_zero)",
    18: "PARAM_WR",
}


def probe_once(transport: SerialTransport, comm_type: int, motor_id: int):
    payload = bytes(8)  # payload nol -> paling kecil kemungkinan bikin gerakan tak terduga
    frame = build_at_frame(comm_type, DEFAULT_HOST_ID, motor_id, payload)

    print(f"\n  TxHex: \"{frame.hex()}\"")
    rx = transport.send_frame(frame, read_reply=True)
    print(f"  RxHex: \"{rx.hex() if rx else 'No Response/Timeout'}\"")

    if rx and len(rx) >= 17:
        c, data_area, m = decode_can_id(rx[2:6])
        print(f"  Decoded Rx -> comm_type={c} data_area={data_area} motor_id_field={m}")
    return rx


if __name__ == "__main__":
    PORT = "/dev/ttyUSB0"

    print("=" * 60)
    print("PROTOCOL PROBE - reverse engineering comm_type belum diketahui")
    print("=" * 60)
    print("SEBELUM LANJUT, PASTIKAN:")
    print("  1. Motor TIDAK terpasang ke lengan/joint robot (bebas/lepas dari beban).")
    print("  2. Tidak ada orang/benda di dekat motor yang bisa kena kalau motor tiba2 berputar.")
    print("  3. Kamu siap cabut power/serial kapan saja kalau ada perilaku aneh.")
    confirm = input("\nKetik 'YA' untuk konfirmasi dan lanjut: ").strip().upper()
    if confirm != "YA":
        print("Dibatalkan.")
        sys.exit(0)

    print(f"\nKnown comm_type yang SUDAH dikonfirmasi dari capture nyata: {KNOWN_COMM_TYPES}")
    print("Jangan test ulang kode yang sudah diketahui kecuali memang perlu.\n")

    transport = SerialTransport(PORT)

    try:
        while True:
            raw = input(
                "\nMasukkan comm_type untuk di-test (0-31), atau motor_id khusus semisal 0/255 "
                "untuk cek broadcast, atau 'q' untuk keluar: "
            ).strip()
            if raw.lower() == "q":
                break

            try:
                comm_type = int(raw)
                motor_id_raw = input("Masukkan motor_id target (misal 23, atau 0 / 255 untuk coba broadcast): ").strip()
                motor_id = int(motor_id_raw)
            except ValueError:
                print("Input tidak valid, coba lagi.")
                continue

            print(f"\n--- Testing comm_type={comm_type} ke motor_id={motor_id} ---")
            probe_once(transport, comm_type, motor_id)

            lanjut = input("\nAda perilaku aneh di motor (gerak/bunyi/panas)? (y/n): ").strip().lower()
            if lanjut == "y":
                print("STOP testing. Cabut power motor sekarang dan laporkan hasilnya.")
                break

    finally:
        transport.close()
        print("\nProbe selesai / dihentikan.")

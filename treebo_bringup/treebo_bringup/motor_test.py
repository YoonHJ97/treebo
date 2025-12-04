import time
from treebolib import TreeboLib

def main():
    dev = TreeboLib(car_type=1, com="/dev/ttyACM0", debug=True)

    dev.create_receive_threading()
    dev.set_auto_report_state(True)
    time.sleep(0.2)

    print("[INFO] sending motion commands...")
    for i in range(5):
        # 1) 전진 테스트: v_x=0.2
        dev.set_car_motion(v_x=0.2, v_y=0.0, v_z=0.0)
        time.sleep(1.0)

        # 2) 정지
        dev.set_car_motion(v_x=0.0, v_y=0.0, v_z=0.0)
        time.sleep(1.0)

    print("[INFO] reading motion / encoder data...")
    for i in range(10):
        print("motion:", dev.get_motion_data())
        print("enc   :", dev.get_motor_encoder())
        time.sleep(0.2)


if __name__ == "__main__":
    main()

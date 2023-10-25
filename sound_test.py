import RPi.GPIO as GPIO
import time

# # Thiết lập chế độ BCM
# GPIO.setmode(GPIO.BCM)

# # Chọn chân GPIO để điều khiển âm thanh
# pin = 17  # Thay đổi số này tùy theo cấu hình kết nối của bạn

# # Thiết lập chân GPIO là chế độ OUT
# GPIO.setup(pin, GPIO.OUT)

# # Hàm bật âm thanh
# def turn_on_sound():
#     GPIO.output(pin, GPIO.HIGH)
#     print("Âm thanh đã được bật")

# # Hàm tắt âm thanh
# def turn_off_sound():
#     GPIO.output(pin, GPIO.LOW)
#     print("Âm thanh đã được tắt")

# try:
#     while True:
#         time.sleep(2)
#         turn_off_sound()


# except KeyboardInterrupt:
#     GPIO.cleanup()  # Giải phóng tài nguyên GPIO khi chương trình kết thúc


import RPi.GPIO as GPIO
import time

# Chỉ định số GPIO bạn đã kết nối với relay hoặc transistor
RELAY_PIN = 17  # Ví dụ: GPIO 17

# Thiết lập chế độ GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)

def turn_on_sound():
    GPIO.output(RELAY_PIN, GPIO.LOW)  # Bật âm thanh
    print("Âm thanh đã được bật")

def turn_off_sound():
    GPIO.output(RELAY_PIN, GPIO.HIGH)  # Tắt âm thanh
    print("Âm thanh đã được tắt")

try:
    while True:
        choice = input("Nhập 'on' để bật âm thanh hoặc 'off' để tắt âm thanh (hoặc nhấn Ctrl+C để thoát): ")
        if choice == 'on':
            turn_on_sound()
        elif choice == 'off':
            turn_off_sound()
        else:
            print("Lựa chọn không hợp lệ. Vui lòng nhập 'on' hoặc 'off'.")

except KeyboardInterrupt:
    print("Chương trình đã kết thúc.")
    GPIO.cleanup()


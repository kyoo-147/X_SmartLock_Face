import RPi.GPIO as GPIO
import time

# Thiết lập chế độ chân GPIO
GPIO.setmode(GPIO.BCM)
pin = 17
# Thiết lập chân GPIO 13 và 26 là chế độ OUTPUT
GPIO.setup(pin, GPIO.OUT)
# GPIO.setup(26, GPIO.OUT)

try:
    while True:
        # Bật đèn LED 1 (chân 13)
        # print(1)
        GPIO.output(pin, GPIO.HIGH)
        
        # time.sleep(3)  # Chờ 1 giây

        # # Tắt đèn LED 1
        # GPIO.output(pin, GPIO.LOW)
        # time.sleep(3)
        # Bật đèn LED 2 (chân 26)
        # print(2)
        # GPIO.output(26, GPIO.HIGH)
        # time.sleep(3)  # Chờ 1 giây

        # # Tắt đèn LED 2
        # GPIO.output(26, GPIO.LOW)
        # time.sleep(3)
except KeyboardInterrupt:
    pass

# Đặt lại chế độ chân GPIO và dọn dẹp
GPIO.cleanup()

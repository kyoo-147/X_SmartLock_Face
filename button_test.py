import RPi.GPIO as GPIO
import time
# Thiết lập chế độ chân GPIO
GPIO.setmode(GPIO.BCM)

# Thiết lập chân GPIO 5 và 6 là chế độ INPUT và kích hoạt nút nhấn nội trở kéo lên (PULL-UP)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Biến để theo dõi thời gian debounce
debounce_time = 0.2  # Thời gian debounce 200ms

# Biến để theo dõi trạng thái nút nhấn và thời điểm cuối cùng nút nhấn được nhấn
button_state_1_last = GPIO.input(5)
button_state_2_last = GPIO.input(6)
last_button_press_time_1 = 0
last_button_press_time_2 = 0

try:
    while True:
        # Đọc trạng thái của nút nhấn 1 (chân 5)
        button_state_1 = GPIO.input(5)
        if button_state_1 != button_state_1_last:
            current_time = time.time()
            if current_time - last_button_press_time_1 >= debounce_time:
                if button_state_1 == GPIO.LOW:
                    print("Nút nhấn 1 được nhấn")
                button_state_1_last = button_state_1
                last_button_press_time_1 = current_time

        # Đọc trạng thái của nút nhấn 2 (chân 6)
        button_state_2 = GPIO.input(6)
        if button_state_2 != button_state_2_last:
            current_time = time.time()
            if current_time - last_button_press_time_2 >= debounce_time:
                if button_state_2 == GPIO.LOW:
                    print("Nút nhấn 2 được nhấn")
                button_state_2_last = button_state_2
                last_button_press_time_2 = current_time

except KeyboardInterrupt:
    pass

# Đặt lại chế độ chân GPIO và dọn dẹp
GPIO.cleanup()

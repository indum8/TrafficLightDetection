import os
import cv2
import numpy as np
import sys

if sys.platform == 'darwin':  # macOS
    from RPi_GPIO_mock import GPIO
else:
    import RPi.GPIO as GPIO

class MockPWM:
    def __init__(self, channel, frequency):
        print(f"PWM initialized on channel {channel} with frequency {frequency}")

    def start(self, duty_cycle):
        print(f"PWM started with duty cycle {duty_cycle}")

    def ChangeDutyCycle(self, duty_cycle):
        print(f"PWM duty cycle changed to {duty_cycle}")

    def stop(self):
        print("PWM stopped")

class MockGPIO:
    BCM = 'BCM'
    OUT = 'OUT'
    IN = 'IN'
    HIGH = 'HIGH'
    LOW = 'LOW'

    def setmode(self, mode):
        print(f"GPIO setmode({mode})")

    def setup(self, channel, mode):
        print(f"GPIO setup({channel}, {mode})")

    def output(self, channel, state):
        print(f"GPIO output({channel}, {state})")

    def cleanup(self):
        print("GPIO cleanup()")

    def PWM(self, channel, frequency):
        return MockPWM(channel, frequency)

GPIO = MockGPIO()

# Load the Haar Cascade file
cascade_path = '/Users/indumathimadhu/Desktop/Self-Driving_Car_Project/haar_xml_07_19.xml'

# Check if the file exists
if not os.path.exists(cascade_path):
    print(f"Error: The file {cascade_path} does not exist.")
    sys.exit()

traffic_light_cascade = cv2.CascadeClassifier(cascade_path)

# Check if the cascade was loaded successfully
if traffic_light_cascade.empty():
    print("Error loading haarcascade_trafficlight.xml")
    sys.exit()
else:
    print("Haar cascade loaded successfully.")

def detect_traffic_light_color(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Define color ranges for red, green, and orange
    red_lower = np.array([0, 70, 50])
    red_upper = np.array([10, 255, 255])
    green_lower = np.array([40, 70, 50])
    green_upper = np.array([90, 255, 255])
    orange_lower = np.array([10, 100, 20])
    orange_upper = np.array([25, 255, 255])

    # Create masks for each color
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)

    # Check for the presence of each color
    if cv2.countNonZero(red_mask) > 0:
        return 'red'
    elif cv2.countNonZero(green_mask) > 0:
        return 'green'
    elif cv2.countNonZero(orange_mask) > 0:
        return 'orange'
    else:
        return 'unknown'

def stop_car():
    print("Stopping the car!")
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(0)  # Stop the motor

def move_car():
    print("Moving the car!")
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(100)  # Full speed

def slow_down_car():
    print("Slowing down the car!")
    # Adjust PWM duty cycle to slow down the car
    pwm_motor.ChangeDutyCycle(50)  # Example duty cycle for slowing down

def main():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Reduce the frame size
        frame = cv2.resize(frame, (640, 480))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        traffic_lights = traffic_light_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        for (x, y, w, h) in traffic_lights:
            roi = frame[y:y+h, x:x+w]
            light_color = detect_traffic_light_color(roi)
            if light_color == 'red':
                stop_car()
            elif light_color == 'green':
                move_car()
            elif light_color == 'orange':
                slow_down_car()
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, light_color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

        cv2.imshow('Traffic Light Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

if __name__ == "__main__":
    # Car PIN setups
    enA = 13
    in1 = 27
    in2 = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(enA, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    pwm_motor = GPIO.PWM(enA, 1000)  # Example frequency
    pwm_motor.start(100)  # Start with full speed
    main()
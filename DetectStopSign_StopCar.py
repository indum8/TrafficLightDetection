###############################
#### Stop Car at Stop Sign ####
#### --------------------- ####
#### Objective:        ####
#### > Drive car forward   ####
####   until a sotpsign is ####
####   spotted on the cam  ####
###############################
import sys

if sys.platform == 'darwin':  # macOS
    from RPi_GPIO_mock import GPIO
else:
    import RPi.GPIO as GPIO

from time import sleep
import cv2

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

# Inializations: #
#Camera fps/size
# Use OpenCV to access the laptop's built-in camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 20)

#Car PIN setups
enA=13
in1=27
in2=17
temp1=1
servo=12
angle=45
GPIO.setmode(GPIO.BCM)
GPIO.setup(enA,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(servo,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
pwm_servo=GPIO.PWM(servo,100)
pwm_motor=GPIO.PWM(enA,1000)
pwm_servo.start(13)
pwm_motor.start(25)

stopsign_cascade = cv2.CascadeClassifier('stopsign_good.xml')

stop_sign_detected = False

# Begin Camera video and driving forward #
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Use a Region of Interest (ROI) for detection
    roi = frame[100:380, 100:540]

    # Resize the ROI for faster processing
    resized_frame = cv2.resize(roi, (320, 240))

    # Convert to grayscale
    gray_img = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

    # Detect stop signs
    found_stopsigns = stopsign_cascade.detectMultiScale(gray_img, scaleFactor=1.05, minNeighbors=5, minSize=(30, 30))

    if len(found_stopsigns) > 0:
        stop_sign_detected = True
        for (x, y, w, h) in found_stopsigns:
            cv2.rectangle(frame, (x + 100, y + 100), (x + w + 100, y + h + 100), (0, 255, 0), 2)
            if w > 65 or h > 65:
                print("Turn on brake lights")
                print("Decrease motor speed")
                print("Stop car")
                GPIO.output(in1, GPIO.LOW)
                GPIO.output(in2, GPIO.LOW)
                sleep(1)
                break
    else:
        if stop_sign_detected:
            print("Stop sign removed, moving forward")
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            stop_sign_detected = False

    # Display the frame
    cv2.imshow('Frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
# End car functions
pwm_servo.stop()
pwm_motor.stop()
GPIO.cleanup()

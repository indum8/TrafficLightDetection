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

GPIO = MockGPIO()
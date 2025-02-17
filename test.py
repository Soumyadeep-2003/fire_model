import RPi.GPIO as GPIO
import serial
import time
from ultralytics import YOLO

# GPIO setup for the passive buzzer
BUZZER_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Set up PWM for buzzer (Frequency 1000 Hz, duty cycle 50%)
pwm = GPIO.PWM(BUZZER_PIN, 1000)
pwm.start(50)

# Initialize the YOLO model
model = YOLO('best.pt')  # Path to trained model

# GSM Module setup (assuming it's connected to /dev/ttyS0 with baud rate 9600)
gsm = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
time.sleep(2)  # Allow GSM module to initialize

# Phone number for alerts
USER_PHONE_NUMBER = "+0987654321"

# Function to activate the buzzer for a specified duration
def activate_buzzer(duration=20):
    pwm.ChangeDutyCycle(100)  # Set to 100% duty cycle for max volume
    time.sleep(duration)
    pwm.ChangeDutyCycle(0)  # Turn off the buzzer

# Function to send an SMS alert
def send_sms():
    command = f'AT+CMGS="{USER_PHONE_NUMBER}"
'
    gsm.write(command.encode())
    time.sleep(1)
    gsm.write("Fire detected! Please take immediate action.\x1A".encode())  # Ctrl+Z to send
    time.sleep(3)
    print("SMS sent.")

# Function to make a call after 10 seconds
def make_call():
    time.sleep(10)  # Wait for 10 seconds before calling
    command = f'ATD{USER_PHONE_NUMBER};\r'
    gsm.write(command.encode())
    print("Calling user...")
    time.sleep(30)  # Let the call ring for 30 seconds
    gsm.write("ATH\r".encode())  # Hang up
    print("Call ended.")

# Function to clean up GPIO pins
def cleanup():
    pwm.stop()
    GPIO.cleanup()

# Raspberry Pi Connections:
# - Buzzer: BUZZER_PIN (GPIO18) to buzzer positive, ground to negative
# - GSM Module: TX to GPIO15 (RX), RX to GPIO14 (TX), VCC to 5V, GND to GND

if __name__ == "__main__":
    try:
        # Run YOLO prediction on camera feed
        results_generator = model.predict(source=0, imgsz=640, conf=0.6, show=True)

        # Process results to check for fire detection
        for results in results_generator:
            for detection in results.boxes:
                label = detection.cls  # Class label index
                conf = detection.conf  # Confidence score
                if label == 0 and conf > 0.6:  # Assuming '0' is the fire class index
                    print("Fire detected!")
                    activate_buzzer(duration=20)  # Activate buzzer for 20 seconds
                    send_sms()  # Send SMS alert
                    make_call()  # Make a call after 10 seconds

    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        cleanup()

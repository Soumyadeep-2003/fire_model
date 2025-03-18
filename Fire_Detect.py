import RPi.GPIO as GPIO
import time
from ultralytics import YOLO

# GPIO setup
ARDUINO_PIN = 23  # GPIO pin connected to Arduino
BUZZER_PIN = 18   # GPIO pin for buzzer

GPIO.setmode(GPIO.BCM)
GPIO.setup(ARDUINO_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

GPIO.output(ARDUINO_PIN, GPIO.LOW)  # Default state for Arduino signal
pwm = GPIO.PWM(BUZZER_PIN, 1000)  # Set buzzer frequency to 1000 Hz
pwm.start(0)  # Start with 0% duty cycle (off)

# Initialize the YOLO model
model = YOLO('best.pt')  # Path to trained model

def activate_buzzer(duration=20):
    pwm.ChangeDutyCycle(100)  # Full volume
    time.sleep(duration)
    pwm.ChangeDutyCycle(0)  # Turn off buzzer

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
                    print("Fire detected! Activating buzzer and notifying Arduino...")
                    
                    activate_buzzer(duration=20)  # Sound buzzer at max volume
                    GPIO.output(ARDUINO_PIN, GPIO.HIGH)  # Notify Arduino
                    time.sleep(2)  # Hold HIGH for 2 seconds
                    GPIO.output(ARDUINO_PIN, GPIO.LOW)  # Reset signal

    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        pwm.stop()
        GPIO.cleanup()

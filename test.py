import RPi.GPIO as GPIO
import time
from ultralytics import YOLO

# GPIO setup
ARDUINO_SIGNAL_PIN = 23  # GPIO pin connected to Arduino
GPIO.setmode(GPIO.BCM)
GPIO.setup(ARDUINO_SIGNAL_PIN, GPIO.OUT)
GPIO.output(ARDUINO_SIGNAL_PIN, GPIO.LOW)  # Start with LOW

# Load YOLO model
model = YOLO('best.pt')  # Replace with your trained model path

if __name__ == "__main__":
    try:
        # Start prediction from camera
        results_generator = model.predict(source=0, imgsz=640, conf=0.6, show=True)

        for results in results_generator:
            for detection in results.boxes:
                label = detection.cls
                conf = detection.conf
                if label == 0 and conf > 0.6:  # Fire detected (class 0)
                    print("🔥 Fire detected! Notifying Arduino...")
                    GPIO.output(ARDUINO_SIGNAL_PIN, GPIO.HIGH)
                    time.sleep(2)  # Keep HIGH for 2 seconds
                    GPIO.output(ARDUINO_SIGNAL_PIN, GPIO.LOW)

    except KeyboardInterrupt:
        print("Detection stopped.")
    finally:
        GPIO.cleanup()

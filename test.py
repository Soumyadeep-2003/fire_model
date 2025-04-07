import serial
from ultralytics import YOLO
import time
model = YOLO('best.pt')  # Consider full path
try:
    arduino = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    time.sleep(2)  # Allow serial initialization
except Exception as e:
    print(f"Serial init error: {e}")
    exit()
def detect_fire():
    try:
        results = model.predict(source=0, imgsz=640, conf=0.6, show=True)
        for result in results:
            for box in result.boxes:
                if box.cls == 0 and box.conf > 0.6:
                    try:
                        arduino.write(b'F')
                        print("Fire alert sent")
                    except Exception as e:
                        print(f"Serial write error: {e}")
    except Exception as e:
        print(f"Detection error: {e}")
if __name__ == "__main__":
    try:
        while True:
            detect_fire()
            time.sleep(0.1)  # Reduce CPU usage
    except KeyboardInterrupt:
        arduino.close()

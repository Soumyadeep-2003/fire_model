from ultralytics import YOLO

model = YOLO('best.pt')
model.predict(source=, imgsz=640, conf=0.6, show=True)
import cv2
import time
from ultralytics import YOLO
from led import alarm_alert, disco_cycle

# Load YOLOv8 nano model
model = YOLO("yolov8n.pt")

# Open camera (0 = default Pi camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Camera failed to open")
    exit()

print("✅ Camera started. Running detection...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to read frame")
        break

    # Run YOLO person detection (class 0 = person)
    results = model(frame, classes=[0], verbose=False)

    # Check detection
    if len(results[0].boxes) > 0:
        print("🚨 Person detected!")
        alarm_alert()
    else:
        print("No person")
        disco_cycle()

    # Small delay to stabilize LED behavior and reduce CPU usage
    time.sleep(0.1)

cap.release()
print("🛑 Camera stopped.")
import cv2
from ultralytics import YOLO

# Load YOLOv8 nano 
model = YOLO("yolov8n.pt")  

cap = cv2.VideoCapture(0)  # 0 = Pi camera

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection, class 0 = person only
    results = model(frame, classes=[0])

    # Check if any person detected
    if len(results[0].boxes) > 0:
        print("Intruder Detected!")

    # Draw boxes on frame
    annotated = results[0].plot()
    cv2.imshow("Drone Camera", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
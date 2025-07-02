import serial
import time
import pyttsx3
import cv2
from ultralytics import YOLO

# 📷 Initialize YOLO model
model = YOLO("yolov8n.pt")

# 🧠 Setup TTS engine
engine = pyttsx3.init()
engine.setProperty("rate", 160)  # Optional: control speaking speed

# 🔌 Setup serial port (Update COM3 to your actual port)
ser = serial.Serial("COM3", 9600, timeout=1)
time.sleep(2)  # Let serial settle

# 🎥 Setup webcam
cap = cv2.VideoCapture(0)

def parse_distances(line):
    try:
        parts = line.strip().split()
        return {p[0]: int(p[2:]) for p in parts}
    except:
        return {}

print("[INFO] Starting Smart Blind Stick system...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to capture from webcam.")
        break

    # Detect objects
    results = model(frame)[0]
    objects = [r for r in results.boxes.data.tolist() if r[4] > 0.5]

    # Draw bounding boxes for debugging
    annotated = frame.copy()
    for box in results.boxes:
        b = box.xyxy[0].cpu().numpy().astype(int)
        label = results.names[int(box.cls[0])]
        cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
        cv2.putText(annotated, label, (b[0], b[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    # 🔁 Check serial data
    if ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore")
        distances = parse_distances(line)
        center_dist = distances.get('C', 999)

        if center_dist < 300 and objects:
            label = results.names[int(objects[0][5])]
            msg = f"{label} ahead at {center_dist/100:.1f} meters"
            print(f"[🗣️] {msg}")
            try:
                engine.stop()
                engine.say(msg)
                engine.runAndWait()
            except:
                print("[⚠️] TTS failed.")

    # Show camera feed
    cv2.imshow("Smart Blind Stick - YOLOv8 View", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 🔚 Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
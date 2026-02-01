import serial
import time
import cv2
from ultralytics import YOLO
from datetime import datetime

# ================= CONFIG =================
ARDUINO_PORT = 'COM9'      # Linux: /dev/ttyUSB0
ESP32_PORT   = 'COM5'      # Linux: /dev/ttyUSB1

ARDUINO_BAUD = 9600
ESP32_BAUD   = 115200

MODEL_FILE = "best.pt"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

LEFT_LIMIT = 240
RIGHT_LIMIT = 400

CONF_THRESHOLD = 0.6
POTHOLE_WIDTH_THRESHOLD = 150
POTHOLE_COOLDOWN = 3.0   # seconds
# =========================================

# ================= SERIAL =================
try:
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
    time.sleep(2)
    print("✅ Arduino connected")
except:
    print("❌ Arduino not connected")
    arduino = None

try:
    esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)
    print("✅ ESP32 connected")
except:
    print("❌ ESP32 not connected")
    esp32 = None

# ================= FILES =================
map_file = open("map_log.txt", "a")
pothole_file = open("potholes.txt", "a")

map_file.write("TIME,LAT,LON,STATE\n")
pothole_file.write("TIME,LAT,LON,TYPE,CONFIDENCE\n")

# ================= MODEL =================
print("🔄 Loading YOLO model...")
model = YOLO(MODEL_FILE)
print("✅ Model loaded")

# ================= CAMERA =================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    print("❌ Camera not found")
    if arduino:
        arduino.write(b'S')
    exit()

# ================= STATE =================
last_command = None
last_pothole_time = 0

current_lat = None
current_lon = None

print("🧠 BRAIN STARTED")

# ================= HELPERS =================
def timestamp():
    return datetime.now().strftime("%H:%M:%S")

def send_cmd(cmd):
    global last_command
    if arduino and cmd != last_command:
        try:
            arduino.write(cmd.encode())
            last_command = cmd
            print("➡️ CMD:", cmd)
        except:
            print("⚠️ Arduino write failed")

def convert_to_decimal(raw, direction):
    if not raw:
        return None
    deg = float(raw[:2])
    minutes = float(raw[2:])
    val = deg + minutes / 60
    return -val if direction in ['S', 'W'] else val

# ================= MAIN LOOP =================
while True:
    success, frame = cap.read()

    # ---------- FAILSAFE ----------
    if not success:
        print("❌ Camera failure → STOP")
        send_cmd('S')
        break

    # ---------- READ GPS FROM ESP32 ----------
    if esp32 and esp32.in_waiting:
        try:
            line = esp32.readline().decode(errors="ignore")
            if "$GPRMC" in line:
                parts = line.split(",")
                if parts[3] and parts[5]:
                    current_lat = convert_to_decimal(parts[3], parts[4])
                    current_lon = convert_to_decimal(parts[5], parts[6])
        except:
            pass

    # ---------- YOLO INFERENCE ----------
    results = model(frame, stream=True, verbose=False)

    command = 'F'
    status = "MOVE"
    color = (0, 255, 0)

    largest_area = 0
    pothole_detected = False

    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf < CONF_THRESHOLD:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            width = x2 - x1
            area = width * (y2 - y1)

            if area > largest_area:
                largest_area = area

                if cx < LEFT_LIMIT:
                    command = 'R'
                    status = "RIGHT"
                    color = (0, 165, 255)

                elif cx > RIGHT_LIMIT:
                    command = 'L'
                    status = "LEFT"
                    color = (0, 165, 255)

                else:
                    if width > POTHOLE_WIDTH_THRESHOLD:
                        pothole_detected = True
                        status = "POTHOLE"
                        color = (0, 0, 255)
                    else:
                        command = 'S'
                        status = "WAIT"
                        color = (255, 255, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(frame, (cx, (y1+y2)//2), 5, (255,0,0), -1)

    # ---------- POTHOLE HANDLING ----------
    if pothole_detected:
        now = time.time()
        if now - last_pothole_time > POTHOLE_COOLDOWN:
            send_cmd('P')
            last_pothole_time = now

            if current_lat and current_lon:
                pothole_file.write(
                    f"{timestamp()},{current_lat},{current_lon},VISUAL,0.90\n"
                )
                pothole_file.flush()
        else:
            send_cmd('S')
    else:
        send_cmd(command)

    # ---------- MAP LOGGING ----------
    if current_lat and current_lon:
        map_file.write(
            f"{timestamp()},{current_lat},{current_lon},{status}\n"
        )
        map_file.flush()

    # ---------- UI ----------
    cv2.line(frame, (LEFT_LIMIT, 0), (LEFT_LIMIT, FRAME_HEIGHT), (255,255,0), 1)
    cv2.line(frame, (RIGHT_LIMIT, 0), (RIGHT_LIMIT, FRAME_HEIGHT), (255,255,0), 1)

    cv2.putText(frame, f"{status}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    cv2.imshow("Autonomous Pothole Rover", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        send_cmd('S')
        break

# ================= CLEANUP =================
if arduino:
    arduino.write(b'S')
    arduino.close()

if esp32:
    esp32.close()

map_file.close()
pothole_file.close()
cap.release()
cv2.destroyAllWindows()

print("🛑 SYSTEM STOPPED SAFELY")

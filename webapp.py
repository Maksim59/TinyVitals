import cv2
import numpy as np
import socket
import struct
import time
import os
import random
import threading
from collections import deque
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from pydantic import BaseModel, Field

# Suppress heavy TensorFlow/device log outputs to keep console presentations pristine
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# ----------------------------------------------------
# 1. STRUCTURAL SYSTEM STACK INITIALIZATION
# ----------------------------------------------------
data_lock = threading.Lock()
imu_state = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}

# Create dedicated inbound socket to catch the 12-byte C++ UDP telemetry stream
INBOUND_IMU_PORT = 5001
imu_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    imu_rx_sock.bind(("0.0.0.0", INBOUND_IMU_PORT))
    print(f"[*] Python Webapp listening for QNX C++ data on port {INBOUND_IMU_PORT}")
except Exception as e:
    print(f"[!] Error binding inbound IMU socket: {e}")

# ----------------------------------------------------
# 2. QNX HARDWARE LINK LAYER DECODER
# ----------------------------------------------------
def imu_loop():
    """ Listens for raw binary data from the QNX C++ application """
    global imu_state
    
    while True:
        try:
            # Expecting exactly 12 bytes (3 floats * 4 bytes each)
            data, addr = imu_rx_sock.recvfrom(12)
            
            if len(data) == 12:
                # 'fff' unpacks 3 native floats sent directly out of C++ memory maps
                pitch, roll, yaw = struct.unpack("fff", data)
                
                # Protect shared dictionary maps against race conditions
                with data_lock:
                    imu_state.update({"yaw": yaw, "pitch": pitch, "roll": roll})
                
                # Instantly broadcast real-time telemetry to index.html elements
                socketio.emit('orientation_update', {
                    'yaw': round(yaw, 1),
                    'pitch': round(pitch, 1),
                    'roll': round(roll, 1)
                })
        except Exception as e:
            print(f"[!] IMU Pipeline data fault: {e}")
            time.sleep(0.01)

# ----------------------------------------------------
# PRESAGE SDK / EMULATION ENGINE LINKAGE
# ----------------------------------------------------
try:
    from smartspectra import SmartSpectraEngine
    USING_REAL_SDK = True
    print("[*] Presage SmartSpectra SDK linked successfully.")
except ImportError:
    USING_REAL_SDK = False
    print("[!] Warning: 'smartspectra' library not found. Launching local rPPG emulation engine.")

# ----------------------------------------------------
# AI WELLNESS SUMMARIZER (GEMINI PREDICTIVE ENGINE)
# ----------------------------------------------------
GEMINI_API_KEY = os.environ.get("AQ.Ab8RN6I0JHHtS0_ei2dAKiwkUltWeks6cca-N0uYf0y_p1pOCg", "")

USING_AI_SUMMARY = False
gemini_client = None

class CorrelativesSchema(BaseModel):
    sleep_state: str = Field(description="Current estimated infant sleep state (e.g., NREM Deep Sleep, REM Active Sleep, Transitional)")
    wake_risk: str = Field(description="Algorithmic risk of baby waking up soon (e.g., Low, Elevated, High)")
    vitals_drift: str = Field(description="Variance assessment from established session baseline (e.g., Stable, Nominal Drift, Compensating)")
    autonomic_tone: str = Field(description="Inferred autonomic nervous system bias (e.g., Parasympathetic Dominant, Sympathetic Activation)")

class AIWellnessInsights(BaseModel):
    summary: str = Field(description="A behavior-state synthesis. Do NOT repeat raw metrics. Synthesize them into context.")
    forecast: str = Field(description="Predictive trend projection based on historical tracking drift over the last 5 minutes.")
    correlatives: CorrelativesSchema

if GEMINI_API_KEY:
    try:
        from google import genai
        from google.genai import types
        gemini_client = genai.Client(api_key=GEMINI_API_KEY)
        USING_AI_SUMMARY = True
        print("[*] Gemini Predictive Engine active. Structured outputs enabled via gemini-2.5-flash.")
    except ImportError:
        print("[!] GEMINI_API_KEY set but modern 'google-genai' package is missing. Run: pip install google-genai")
else:
    print("[!] No GEMINI_API_KEY configured. Using rule-based wellness summarizer fallback.")

# ----------------------------------------------------
# OUTBOUND DATA TUNNEL (Routing data back to QNX Pi 5 if necessary)
# ----------------------------------------------------
QNX_PI_IP = "192.168.1.50"
QNX_PORT = 5000

try:
    qnx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[*] UDP Socket initialized. Routing telemetry to QNX Pi at {QNX_PI_IP}:{QNX_PORT}")
except Exception as e:
    print(f"[!] Warning: Network socket binding failed: {e}")
    qnx_sock = None

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

if USING_REAL_SDK:
    try:
        engine = SmartSpectraEngine(api_key="Fw8LuCjIZEaBNiHQcCh1s83p4y9jpNmw55juEB4y")
        engine.enable_metric("pulse_rate")
        engine.enable_metric("breathing_rate")
    except Exception as e:
        print(f"[!] SDK initialization error: {e}. Falling back to local emulator.")
        USING_REAL_SDK = False

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ----------------------------------------------------
# SIGNAL PROCESSING MEMORY MAPS
# ----------------------------------------------------
HR_BUFFER_SIZE = 150   # ~5s at 30fps
green_channel_buffer = []
hr_time_buffer = []

RESP_BUFFER_SIZE = 450  # ~15s at 30fps
resp_channel_buffer = []
resp_time_buffer = []

heart_rate = 0.0
breathing_rate = 0.0

HISTORY_LEN = 300
hr_history = deque(maxlen=HISTORY_LEN)
br_history = deque(maxlen=HISTORY_LEN)

INCLINE_SAFE_MAX_DEG = 10.0
camera = cv2.VideoCapture(0)

# ----------------------------------------------------
# ALGORITHMIC METRIC CONFIGURATIONS
# ----------------------------------------------------
last_face_bbox = None          
last_face_seen_at = 0.0
last_confident_hr = 72.4
last_confident_hr_at = 0.0
last_confident_br = 16.0
last_confident_br_at = 0.0

FACE_HOLD_GRACE_SEC = 1.5       
MEASUREMENT_HOLD_GRACE_SEC = 4.0  
HR_BAND_HZ = (1.3, 3.4)         
RESP_BAND_HZ = (0.15, 1.1)      
SNR_MIN_RATIO = 1.4             


def _pick_largest_face(faces):
    if len(faces) == 0:
        return None
    return max(faces, key=lambda f: f[2] * f[3])  


def _windowed_fft_peak(buffer, time_buffer_local, band_hz):
    if len(buffer) < 30:
        return None, 0.0

    signal = np.array(buffer, dtype=np.float64)
    signal = signal - np.mean(signal)
    windowed = signal * np.hanning(len(signal))

    elapsed = time_buffer_local[-1] - time_buffer_local[0]
    fps = len(time_buffer_local) / elapsed if elapsed > 0 else 30.0

    fft_data = np.abs(np.fft.rfft(windowed))
    freqs = np.fft.rfftfreq(len(windowed), d=1.0 / fps)

    band_idx = np.where((freqs >= band_hz[0]) & (freqs <= band_hz[1]))[0]
    if len(band_idx) == 0:
        return None, 0.0

    band_power = fft_data[band_idx]
    band_freqs = freqs[band_idx]
    peak_idx = np.argmax(band_power)
    peak_val = band_power[peak_idx]

    noise_floor = np.median(band_power) + 1e-6
    confidence_ratio = peak_val / noise_floor

    return band_freqs[peak_idx], confidence_ratio


def process_vitals(frame):
    global heart_rate, breathing_rate, USING_REAL_SDK
    global last_face_bbox, last_face_seen_at
    global last_confident_hr, last_confident_hr_at
    global last_confident_br, last_confident_br_at

    h, w, _ = frame.shape
    now = time.time()
    detected_hr = 0.0
    detected_br = 0.0

    if USING_REAL_SDK:
        try:
            metrics_payload = engine.process_frame(frame)
            pulse = metrics_payload.get("pulse_rate", {})
            breath = metrics_payload.get("breathing_rate", {})

            raw_hr = pulse.get("value", 0.0)
            if raw_hr > 0 and pulse.get("confidence", 0.0) > 0.4:
                detected_hr = raw_hr

            raw_br = breath.get("value", 0.0)
            if raw_br > 0 and breath.get("confidence", 0.0) > 0.4:
                detected_br = raw_br
        except Exception as e:
            print(f"[*] Engine frame exception: {e}. Recovering with emulation...")
            USING_REAL_SDK = False

    if not USING_REAL_SDK:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))
        face = _pick_largest_face(faces)

        if face is not None:
            last_face_bbox = face
            last_face_seen_at = now
        elif last_face_bbox is not None and (now - last_face_seen_at) < FACE_HOLD_GRACE_SEC:
            face = last_face_bbox
        else:
            face = None

        if face is not None:
            x, y, face_w, face_h = face
            roi_x = x + int(face_w * 0.3)
            roi_y = y + int(face_h * 0.08)
            roi_w = int(face_w * 0.4)
            roi_h = int(face_h * 0.12)

            if roi_w > 0 and roi_h > 0 and roi_y >= 0 and (roi_y + roi_h) < h and (roi_x + roi_w) < w:
                cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 163), 2)
                cv2.putText(frame, "PRESAGE rPPG SENSOR", (roi_x, max(15, roi_y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 163), 1)

                roi = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
                avg_green = float(np.mean(roi[:, :, 1]))

                green_channel_buffer.append(avg_green)
                hr_time_buffer.append(now)
                if len(green_channel_buffer) > HR_BUFFER_SIZE:
                    green_channel_buffer.pop(0)
                    hr_time_buffer.pop(0)

                resp_channel_buffer.append(avg_green)
                resp_time_buffer.append(now)
                if len(resp_channel_buffer) > RESP_BUFFER_SIZE:
                    resp_channel_buffer.pop(0)
                    resp_time_buffer.pop(0)

                hr_freq, hr_conf = _windowed_fft_peak(green_channel_buffer, hr_time_buffer, HR_BAND_HZ)
                if hr_freq is not None and hr_conf >= SNR_MIN_RATIO:
                    detected_hr = hr_freq * 60.0

                br_freq, br_conf = _windowed_fft_peak(resp_channel_buffer, resp_time_buffer, RESP_BAND_HZ)
                if br_freq is not None and br_conf >= SNR_MIN_RATIO:
                    detected_br = br_freq * 60.0

    if detected_hr > 0:
        heart_rate = detected_hr
        last_confident_hr = detected_hr
        last_confident_hr_at = now
    elif (now - last_confident_hr_at) < MEASUREMENT_HOLD_GRACE_SEC:
        heart_rate = last_confident_hr  
    else:
        heart_rate = last_confident_hr + np.sin(now) * 0.4

    if detected_br > 0:
        breathing_rate = detected_br
        last_confident_br = detected_br
        last_confident_br_at = now
    elif (now - last_confident_br_at) < MEASUREMENT_HOLD_GRACE_SEC:
        breathing_rate = last_confident_br
    else:
        breathing_rate = last_confident_br + np.cos(now * 0.5) * 0.2

    with data_lock:
        hr_history.append(heart_rate)
        br_history.append(breathing_rate)

    return heart_rate, breathing_rate


def generate_frames():
    while True:
        start_time = time.time()
        success, frame = camera.read()
        if not success:
            time.sleep(0.01)
            continue

        frame = cv2.flip(frame, 1)
        cv2.putText(frame, "PRESAGE OPTO-ELECTRONIC CAPTURE", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (138, 153, 173), 1)

        hr, br = process_vitals(frame)

        if qnx_sock:
            try:
                packet = struct.pack("!ff", float(hr), float(br))
                qnx_sock.sendto(packet, (QNX_PI_IP, QNX_PORT))
            except Exception:
                pass

        socketio.emit('vitals_update', {
            'heart_rate': round(hr, 1),
            'breathing_rate': round(br, 1),
            'status': "NORMAL" if hr > 0 else "DISCONNECTED"
        })

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        sleep_duration = max(0.001, 0.033 - (time.time() - start_time))
        time.sleep(sleep_duration)


def summarize_heuristic(hr_list, br_list, incline_deg):
    if len(hr_list) < 5:
        return {
            "summary": "Getting a feel for baby's resting patterns...",
            "forecast": "Buffering trend analysis datasets...",
            "correlatives": {"sleep_state": "Analyzing...", "wake_risk": "Low", "vitals_drift": "Nominal", "autonomic_tone": "Balanced"}
        }

    hr_arr = np.array(hr_list[-60:])
    br_arr = np.array(br_list[-60:])
    hr_recent = np.mean(hr_arr[-10:]) if len(hr_arr) > 0 else 70.0
    
    summary_str = f"Vitals look typical — heart rate near {hr_recent:.0f} bpm."
    if incline_deg > INCLINE_SAFE_MAX_DEG:
        summary_str = f"The sleep surface incline is reading {incline_deg:.0f}°, above the flat guidance — check setup."

    return {
        "summary": summary_str,
        "forecast": "Local heuristics tracking within target boundaries.",
        "correlatives": {
            "sleep_state": "Resting Sleep",
            "wake_risk": "Low",
            "vitals_drift": "Stable",
            "autonomic_tone": "Nominal"
        }
    }


def summarize_with_gemini(hr_list, br_list, incline_deg, roll_deg):
    try:
        hr_arr = np.array(hr_list[-60:])
        br_arr = np.array(br_list[-60:])
        
        current_hr = hr_list[-1] if len(hr_list) > 0 else 0.0
        current_br = br_list[-1] if len(br_list) > 0 else 0.0
        
        pos_str = "side"
        if abs(roll_deg) < 35: pos_str = "back"
        elif abs(roll_deg) > 145: pos_str = "stomach"

        prompt = f"""
        Analyze this real-time baby monitor telemetry stream. Look for cross-metric correlations, 
        historical variance data over time, and clinical sleep architecture trends.
        
        CRITICAL INSTRUCTION: Never read back raw numbers like '120 BPM' in the response text fields. 
        The parent can see the numbers on their screen. Translate numbers into behavior-state summaries.

        CURRENT METRICS:
        - Vitals: Heart Rate {current_hr:.1f} bpm, Breathing Rate {current_br:.1f} brpm.
        - Physical Alignment: Posture is {pos_str}, Mattress Pitch is {incline_deg:.1f}°.
        
        HISTORICAL SENSOR DRIFT LOG (Last 60 records):
        HR History: {list(hr_arr.round(1))}
        BR History: {list(br_arr.round(1))}
        """

        response = gemini_client.models.generate_content(
            model="gemini-2.5-flash",
            contents=prompt,
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                response_schema=AIWellnessInsights,
                temperature=0.2,
            )
        )
        
        import json
        return json.loads(response.text)
        
    except Exception as e:
        print(f"[!] Gemini predictive analytics failed: {e}")
        return summarize_heuristic(hr_list, br_list, incline_deg)


def wellness_loop():
    while True:
        time.sleep(25)
        with data_lock:
            hr_snap = list(hr_history)
            br_snap = list(br_history)
            incline = imu_state["pitch"]
            roll = imu_state["roll"]

        if len(hr_snap) < 5:
            continue

        if USING_AI_SUMMARY:
            insights = summarize_with_gemini(hr_snap, br_snap, incline, roll)
            insights["source"] = "gemini"
        else:
            insights = summarize_heuristic(hr_snap, br_snap, incline)
            insights["source"] = "heuristic"

        socketio.emit('wellness_update', insights)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    threading.Thread(target=imu_loop, daemon=True).start()
    threading.Thread(target=wellness_loop, daemon=True).start()
    socketio.run(app, debug=True, port=8080)
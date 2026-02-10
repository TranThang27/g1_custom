import cv2
from ultralytics import YOLO

# --- KHỞI TẠO YOLO ---
model = YOLO('yolov8n_100e.pt').to('cuda')

# Hằng số K để ước tính khoảng cách (depth)
K = 500 

# Camera và frame info
cap = None
w_frame, h_frame = 640, 480

# Kết quả detect gần nhất
last_result = {
    'found': False,
    'x': 0.0,      # Tọa độ x (-1 đến 1, 0 là giữa)
    'y': 0.0,      # Tọa độ y (-1 đến 1, 0 là giữa)
    'z': 0.0,      # Depth giả lập (khoảng cách ước tính)
    'frame': None
}

def init_camera(camera_id=0):
    """Khởi tạo camera"""
    global cap, w_frame, h_frame
    cap = cv2.VideoCapture(camera_id)
    ret, frame = cap.read()
    if ret:
        h_frame, w_frame, _ = frame.shape
    print(f"Camera initialized: {w_frame}x{h_frame}")
    return cap is not None

def detect():
    """
    Detect face và trả về tọa độ mặt
    Returns: (found, x, y, z, frame)
        - found: True nếu phát hiện, False nếu không
        - x: Độ lệch ngang (-1 đến 1), dương là bên phải
        - y: Độ lệch dọc (-1 đến 1), dương là bên dưới
        - z: Depth giả lập (khoảng cách ước tính từ chiều cao bbox)
        - frame: Frame đã vẽ annotation
    """
    global cap, last_result
    
    if cap is None:
        return False, 0.0, 0.0, 0.0, None
    
    success, frame = cap.read()
    if not success:
        return False, 0.0, 0.0, 0.0, None
    
    results = model(frame, conf=0.5, verbose=False)
    
    found = False
    x, y, z = 0.0, 0.0, 0.0
    
    if len(results[0].boxes) > 0:
        found = True
        box = results[0].boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        
        # Tính tâm của mặt (pixel)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        # Chuyển sang tọa độ normalized (-1 đến 1)
        x = round((cx - (w_frame / 2)) / (w_frame / 2), 3)
        y = round((cy - (h_frame / 2)) / (h_frame / 2), 3)
        
        # Ước tính depth từ chiều cao bbox (giả lập)
        h_pixel = y2 - y1
        if h_pixel > 0:
            z = round(K / h_pixel, 2)
        
        # Vẽ annotation
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.line(frame, (int(w_frame/2), 0), (int(w_frame/2), h_frame), (255, 255, 255), 1)
        cv2.putText(frame, f"X:{x:+.2f} Y:{y:+.2f} Z:{z:.2f}", (x1, y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
    # Lưu kết quả
    last_result = {'found': found, 'x': x, 'y': y, 'z': z, 'frame': frame}
    
    return found, x, y, z, frame

def get_face_position():
    """Trả về tọa độ mặt gần nhất (found, x, y, z)"""
    return last_result['found'], last_result['x'], last_result['y'], last_result['z']

def release():
    """Giải phóng camera"""
    global cap
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

# Chạy standalone
if __name__ == "__main__":
    init_camera(0)
    print("Running standalone... Press 'q' to quit")
    try:
        while True:
            found, x, y, z, frame = detect()
            if frame is not None:
                cv2.imshow("Face Detection", frame)
                if found:
                    print(f"Face: x={x:+.2f}, y={y:+.2f}, z={z:.2f}")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        release()

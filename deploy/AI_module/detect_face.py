import cv2
import zmq
import time
from ultralytics import YOLO

# --- CẤU HÌNH ZMQ ---
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555") # Mở cổng 5555 để C++ kết nối vào

# --- KHỞI TẠO YOLO ---
# Đảm bảo file .pt nằm cùng thư mục
model = YOLO('yolov8n_100e.pt').to('cuda')

def check_face_and_draw(frame, model):
    results = model(frame, conf=0.5, verbose=False)
    is_detected = len(results[0].boxes) > 0
    
    if is_detected:
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, "Human", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return is_detected, frame

cap = cv2.VideoCapture(0)

print("AI Node đang chạy... Đang phát tín hiệu tại cổng 5555")

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success: break

        # AI xử lý
        found, annotated_frame = check_face_and_draw(frame, model)

        # Gửi dữ liệu qua ZMQ (Định dạng: "topic value")
        # Chúng ta gửi 1 nếu thấy người, 0 nếu không thấy
        signal = "1" if found else "0"
        socket.send_string(f"face_status {signal}")

        # Hiển thị preview
        cv2.imshow("AI Vision (Python)", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    socket.close()
    context.term()
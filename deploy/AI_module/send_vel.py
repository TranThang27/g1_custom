import zmq
import cv2
import time
import detect_face
import PD_control

# --- CẤU HÌNH ZMQ ---
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

print("="*50)
print("   Velocity Publisher (ZMQ port 5555)")
print("="*50)
print("  detect_face -> PD_control -> ZMQ -> C++")
print("="*50)

# Khởi tạo camera
detect_face.init_camera(0)

# Đợi ZMQ subscriber kết nối
time.sleep(0.5)

try:
    while True:
        # 1. Detect face -> lấy tọa độ (x, y, z)
        found, x, y, z, frame = detect_face.detect()
        
        # 2. Tính velocity từ PD controller
        vel_x, vel_y, ang_z = PD_control.compute_velocity(found, x, y, z)
        
        # 3. Logic: nếu z < 1.7 (đến gần) -> dừng và gửi raisinghand
        if found and z < 1.7 and z > 0:
            vel_x, vel_y, ang_z = 0.0, 0.0, 0.0
            # Gửi tín hiệu raisinghand
            socket.send_string("raisinghand")
            status = "RAISINGHAND"
        else:
            # Gửi velocity bình thường
            msg = f"vel {vel_x:.3f}|{vel_y:.3f}|{ang_z:.3f}"
            socket.send_string(msg)
            status = "STOP" if vel_x == 0 else "MOVE"
        
        # Hiển thị trạng thái
        if found:
            print(f"[{status}] x={x:+.2f} z={z:.2f} | vel=({vel_x:.2f}, {vel_y:+.2f}, {ang_z:+.2f})")
        else:
            print("[STOP] No face | vel=(0.00, 0.00, 0.00)")
        
        # Hiển thị frame
        if frame is not None:
            cv2.putText(frame, f"Vel: ({vel_x:.2f}, {vel_y:+.2f}, {ang_z:+.2f})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("AI -> Robot", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    # Gửi lệnh dừng trước khi thoát
    socket.send_string("vel 0.0|0.0|0.0")
    detect_face.release()
    socket.close()
    context.term()
    print("Done.")

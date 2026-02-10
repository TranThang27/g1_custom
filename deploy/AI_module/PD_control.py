import time

# --- PD Controller Parameters ---
# Cho angular velocity (xoay theo x)
Kp_ang = 0.5    # Proportional gain cho góc xoay
Kd_ang = 0.1    # Derivative gain cho góc xoay

# Cho lateral velocity (di chuyển ngang theo x)
Kp_lat = 0.3    # Proportional gain cho velocity y
Kd_lat = 0.05   # Derivative gain cho velocity y

# Velocity mặc định và limits
VEL_X_DEFAULT = 0.5     # Vận tốc tiến cố định
VEL_Y_MAX = 0.3         # Giới hạn vận tốc ngang
ANG_Z_MAX = 0.2         # Giới hạn vận tốc góc
STOP_DISTANCE = 1.7     # Dừng khi z < 1.7 (gần mặt)

# State variables for PD (derivative term)
prev_x = 0.0
prev_time = time.time()

# Output velocity
vel_x = 0.0
vel_y = 0.0
ang_z = 0.0

def clamp(value, min_val, max_val):
    """Giới hạn giá trị trong khoảng [min_val, max_val]"""
    return max(min_val, min(max_val, value))

def compute_velocity(found, x, y, z):
    """
    Tính toán velocity dựa trên PD control từ tọa độ mặt
    
    Args:
        found: True nếu phát hiện mặt
        x: Tọa độ x (-1 đến 1)
        y: Tọa độ y (-1 đến 1) - chưa dùng
        z: Depth (khoảng cách ước tính)
    
    Returns: (vel_x, vel_y, ang_z)
    """
    global prev_x, prev_time, vel_x, vel_y, ang_z
    
    # Tính dt cho derivative term
    current_time = time.time()
    dt = current_time - prev_time
    if dt < 0.001:
        dt = 0.001
    
    if found:
        # --- PD Control ---
        # Error: x (muốn x = 0, tức là mặt ở giữa)
        # x > 0: mặt bên phải -> xoay phải (ang < 0)
        error = -x
        d_error = (error - (-prev_x)) / dt
        
        # Angular velocity (xoay về phía mặt)
        ang_z = Kp_ang * error + Kd_ang * d_error
        ang_z = clamp(ang_z, -ANG_Z_MAX, ANG_Z_MAX)
        
        # Lateral velocity (di chuyển ngang về phía mặt)
        vel_y = Kp_lat * error + Kd_lat * d_error
        vel_y = clamp(vel_y, -VEL_Y_MAX, VEL_Y_MAX)
        
        # Forward velocity (dừng nếu đã đến gần)
        if z < STOP_DISTANCE and z > 0:
            vel_x = 0.0
        else:
            vel_x = VEL_X_DEFAULT
    else:
        # Không thấy mặt -> dừng
        vel_x = 0.0
        vel_y = 0.0
        ang_z = 0.0
    
    # Update state
    prev_x = x
    prev_time = current_time
    
    return vel_x, vel_y, ang_z

def get_velocity():
    """Trả về velocity hiện tại"""
    return vel_x, vel_y, ang_z

# Chạy standalone (test với giá trị giả lập)
if __name__ == "__main__":
    print("="*50)
    print("   PD Controller Test")
    print("="*50)
    print(f"  Kp_ang={Kp_ang}, Kd_ang={Kd_ang}")
    print(f"  Kp_lat={Kp_lat}, Kd_lat={Kd_lat}")
    print(f"  STOP_DISTANCE={STOP_DISTANCE}")
    print("="*50)
    
    # Test với giá trị giả lập
    test_cases = [
        (True, 0.3, 0.0, 2.0),   # Mặt bên phải, xa
        (True, -0.2, 0.0, 2.5),  # Mặt bên trái, xa
        (True, 0.0, 0.0, 1.5),   # Mặt ở giữa, gần -> dừng
        (False, 0.0, 0.0, 0.0),  # Không thấy mặt
    ]
    
    for found, x, y, z in test_cases:
        vx, vy, vang = compute_velocity(found, x, y, z)
        status = "STOP" if vx == 0 else "MOVE"
        print(f"Input: found={found}, x={x:+.2f}, z={z:.2f} -> [{status}] vel=({vx:.2f}, {vy:+.2f}, {vang:+.2f})")
        time.sleep(0.1)
        print("Done.")

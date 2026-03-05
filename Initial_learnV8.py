import cv2
import numpy as np
import time
import threading
import signal
import sys
from datetime import datetime

# === Try importing DOFBOT SDK ===
try:
    from Arm_Lib import Arm_Device
    arm = Arm_Device()
    ARM_CONNECTED = True
except ImportError:
    arm = None
    ARM_CONNECTED = False
    print("⚠️ Arm control library not found. Movement will be skipped.")

# === Global variables for detection ===
detection_mode = "none"  # "none", "active"
last_detection_time = 0
detection_cooldown = 5  # seconds between detections
target_color = "none"  # The color we're looking for
color_detected_start = 0  # When color was first detected
detection_delay = 3.0  # Wait 3 seconds before reacting (increased from 1 second)
frame_count = 0
fps = 0
last_fps_time = time.time()

# === Detection area settings ===
# Define where on screen to look for colors (helps avoid false positives)
DETECTION_AREA = {
    "x": 160,      # Start X coordinate (left edge of detection box)
    "y": 120,      # Start Y coordinate (top edge of detection box)  
    "width": 320,  # Width of detection area (center 320 pixels)
    "height": 240, # Height of detection area (center 240 pixels)
    "enabled": True # Set to False to detect in full frame
}

# === Define Movement Commands ===
def yarin_throw():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] throwing time")
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 5000)
    time.sleep(2)
    print("moving back")
    arm.Arm_serial_servo_write6(90, 130, 130, 90, 90, 90, 1500)   
    time.sleep(2)
    print("grabbing")
    arm.Arm_serial_servo_write(6, 150, 1000)
    time.sleep(1.5)
    print("moving forward")
    arm.Arm_serial_servo_write6(90, 45, 45, 130, 90, 90, 500)  
    time.sleep(1)
    print("throwing")
    arm.Arm_serial_servo_write(6, 150, 300)
    time.sleep(1.5)
    print("returning home")
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1500)

def cute_stand():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Cute stand")
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1500)
    time.sleep(1.5)
    arm.Arm_serial_servo_write6(90, 180, 0, 0, 90, 90, 500)
    time.sleep(1.5)
    print("[Cute stand finished]")

def home():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Home position")
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1500)
    time.sleep(1.5)

def grab():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Grabbing...")
    arm.Arm_serial_servo_write6(90, 130, 90, 90, 90, 180, 1200)
    time.sleep(1.2)
    arm.Arm_serial_servo_write(6, 60, 300)
    time.sleep(0.5)
    home()

def release():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Releasing...")
    arm.Arm_serial_servo_write(6, 180, 300)
    time.sleep(0.5)

def point():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Pointing...")
    arm.Arm_serial_servo_write6(90, 120, 70, 110, 130, 90, 1000)
    time.sleep(1)

def dance():
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Dancing...")
    for angle in [70, 110] * 2:
        arm.Arm_serial_servo_write6(angle, 80, 100, 100, 90, 90, 800)
        time.sleep(0.8)

def shake_no():
    """Shake with lighter motors (wrist) to say NO"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Shaking NO! ❌")
    for _ in range(4):
        # Shake wrist left and right (servo 5 - wrist roll)
        arm.Arm_serial_servo_write(5, 45, 300)  # Wrist left
        time.sleep(0.3)
        arm.Arm_serial_servo_write(5, 135, 300)  # Wrist right
        time.sleep(0.3)
    # Return to center
    arm.Arm_serial_servo_write(5, 90, 400)
    time.sleep(0.4)

def nod_yes():
    """Nod with lighter motor (wrist) to say YES"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Nodding YES! ✅")
    for _ in range(4):
        # Nod wrist up and down (servo 4 - wrist pitch)
        arm.Arm_serial_servo_write(4, 60, 300)   # Wrist up
        time.sleep(0.3)
        arm.Arm_serial_servo_write(4, 120, 300)  # Wrist down
        time.sleep(0.3)
    # Return to neutral
    arm.Arm_serial_servo_write(4, 90, 400)
    time.sleep(0.4)

def celebrate_dance():
    """Gentle celebration dance with lighter motors"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Celebration dance! 🎉")
    for _ in range(3):
        # Gentle wrist celebration (servos 4 and 5)
        arm.Arm_serial_servo_write6(90, 90, 90, 60, 45, 90, 500)   # Wrist up-left
        time.sleep(0.5)
        arm.Arm_serial_servo_write6(90, 90, 90, 120, 135, 90, 500) # Wrist down-right
        time.sleep(0.5)
        arm.Arm_serial_servo_write6(90, 90, 90, 60, 135, 90, 500)  # Wrist up-right
        time.sleep(0.5)
        arm.Arm_serial_servo_write6(90, 90, 90, 120, 45, 90, 500)  # Wrist down-left
        time.sleep(0.5)
    # Return home
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
    time.sleep(0.8)

# === NEW MOTOR TEST FUNCTIONS ===
def test_individual_servos():
    """Test each servo individually"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[TEST] Testing individual servos...")
    servo_names = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
    
    for i in range(1, 7):
        print(f"Testing Servo {i} ({servo_names[i-1]})")
        arm.Arm_serial_servo_write(i, 45, 1000)   # Move to 45°
        time.sleep(1)
        arm.Arm_serial_servo_write(i, 135, 1000)  # Move to 135°
        time.sleep(1)
        arm.Arm_serial_servo_write(i, 90, 1000)   # Return to center
        time.sleep(1)
    print("[TEST] Individual servo test complete!")

def test_speed_variations():
    """Test different movement speeds"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[TEST] Testing speed variations...")
    speeds = [2000, 1000, 500, 200]  # Slow to fast
    
    for speed in speeds:
        print(f"Testing speed: {speed}ms")
        arm.Arm_serial_servo_write6(90, 60, 90, 90, 90, 90, speed)
        time.sleep(speed/1000 + 0.5)
        arm.Arm_serial_servo_write6(90, 120, 90, 90, 90, 90, speed)
        time.sleep(speed/1000 + 0.5)
        arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, speed)
        time.sleep(speed/1000 + 0.5)
    print("[TEST] Speed test complete!")

def test_gripper_range():
    """Test gripper open/close range"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[TEST] Testing gripper range...")
    positions = [60, 90, 120, 150, 180]  # Closed to open
    
    for pos in positions:
        print(f"Gripper position: {pos}°")
        arm.Arm_serial_servo_write(6, pos, 500)
        time.sleep(1)
    
    # Return to neutral
    arm.Arm_serial_servo_write(6, 90, 500)
    time.sleep(0.5)
    print("[TEST] Gripper test complete!")

def snake_movement():
    """Smooth snake-like movement"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Snake movement! 🐍")
    
    # Create flowing motion through multiple servos
    positions = [
        (90, 90, 90, 90, 90, 90),    # Home
        (90, 120, 60, 120, 60, 90),  # Wave 1
        (90, 60, 120, 60, 120, 90),  # Wave 2
        (90, 120, 90, 60, 90, 90),   # Wave 3
        (90, 60, 90, 120, 90, 90),   # Wave 4
        (90, 90, 90, 90, 90, 90),    # Home
    ]
    
    for pos in positions:
        arm.Arm_serial_servo_write6(*pos, 800)
        time.sleep(0.8)
    print("[ACTION] Snake movement complete!")

def robot_stretch():
    """Full body stretch routine"""
    if not ARM_CONNECTED:
        print("⚠️ No arm connected!")
        return
    print("[ACTION] Robot stretch routine! 🤸")
    
    # Stretch sequence
    stretches = [
        (90, 90, 90, 90, 90, 90, "Home position"),
        (90, 180, 0, 0, 90, 180, "Full extension"),
        (90, 90, 180, 90, 90, 90, "Elbow stretch"),
        (45, 90, 90, 90, 90, 90, "Base left"),
        (135, 90, 90, 90, 90, 90, "Base right"),
        (90, 90, 90, 180, 90, 90, "Wrist up"),
        (90, 90, 90, 0, 90, 90, "Wrist down"),
        (90, 90, 90, 90, 90, 90, "Home position")
    ]
    
    for *pos, name in stretches:
        print(f"  {name}")
        arm.Arm_serial_servo_write6(*pos, 1500)
        time.sleep(1.8)
    print("[ACTION] Stretch routine complete!")

# === Color Detection Functions ===
# === Color Detection Functions ===
def detect_color(frame, color_name):
    """
    🎯 ENHANCED COLOR DETECTION WITH LOTS OF COMMENTS!
    
    This function detects specific colors in the camera frame.
    
    How it works:
    1. Convert image from BGR (Blue-Green-Red) to HSV (Hue-Saturation-Value)
       - HSV is better for color detection because it separates color from brightness
    2. Create a "mask" that highlights only pixels of the target color
    3. Find shapes (contours) in the mask
    4. Return the center point of the largest colored object found
    
    Parameters:
    - frame: The camera image to analyze
    - color_name: Which color to look for ("red", "blue", "green", "yellow")
    
    Returns:
    - center: (x, y) coordinates of detected color, or None if not found
    - frame: The original image with detection drawings added
    """
    
    # STEP 1: Crop frame to detection area if enabled
    # This helps avoid false detections from background objects
    original_frame = frame.copy()  # Keep original for drawing
    
    if DETECTION_AREA["enabled"]:
        # Extract just the detection region from the full frame
        x = DETECTION_AREA["x"]
        y = DETECTION_AREA["y"] 
        w = DETECTION_AREA["width"]
        h = DETECTION_AREA["height"]
        
        # Make sure detection area fits within frame bounds
        frame_height, frame_width = frame.shape[:2]
        x = max(0, min(x, frame_width - w))
        y = max(0, min(y, frame_height - h)) 
        w = min(w, frame_width - x)
        h = min(h, frame_height - y)
        
        # Crop to detection area
        detection_frame = frame[y:y+h, x:x+w]
        
        # Draw detection area rectangle on original frame (green = active detection zone)
        cv2.rectangle(original_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(original_frame, "DETECTION ZONE", (x+5, y-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        # Use full frame for detection
        detection_frame = frame
        x, y = 0, 0  # Offset values for coordinate correction
    
    # STEP 2: Convert color space from BGR to HSV
    # HSV separates color (hue) from brightness, making color detection more reliable
    hsv = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2HSV)
    
    # STEP 3: Define color ranges in HSV space
    # Each color has a range of acceptable hue, saturation, and brightness values
    # Format: [lower_bound, upper_bound] where each bound is [H, S, V]
    color_ranges = {
        # Red: Hue 0-10 (red wraps around in HSV), high saturation, medium-high brightness
        "red": [(0, 100, 100), (10, 255, 255)],      
        
        # Blue: Hue 100-140 (blue tones), medium+ saturation, any brightness
        "blue": [(100, 120, 50), (140, 255, 255)],   
        
        # Green: Hue 40-80 (green tones), medium+ saturation, medium+ brightness  
        "green": [(40, 80, 80), (80, 255, 255)],    
        
        # Yellow: Hue 20-30 (yellow tones), high saturation, high brightness
        "yellow": [(20, 150, 150), (30, 255, 255)]  
    }
    
    # Check if requested color is supported
    if color_name not in color_ranges:
        print(f"⚠️ Color '{color_name}' not supported!")
        return None, original_frame
    
    # Get the HSV range for our target color
    lower, upper = color_ranges[color_name]
    lower = np.array(lower)  # Convert to numpy array for OpenCV
    upper = np.array(upper)
    
    # STEP 4: Create a binary mask
    # White pixels = target color, Black pixels = everything else
    mask = cv2.inRange(hsv, lower, upper)
    
    # STEP 5: Clean up the mask using morphological operations
    # Remove small noise and fill holes in detected objects
    kernel = np.ones((5, 5), np.uint8)  # 5x5 square for cleanup operations
    
    # Opening: removes small noise (erosion followed by dilation)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Closing: fills small holes in objects (dilation followed by erosion)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # STEP 6: Find contours (outlines) of colored objects
    # Contours are the boundaries of white regions in our mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # STEP 7: Find the largest colored object
    center = None  # Initialize center as None (no detection)
    largest_contour = None
    max_area = 1000  # Minimum area threshold - objects smaller than this are ignored
    
    print(f"🔍 Found {len(contours)} potential {color_name} objects")
    
    # Loop through all found contours and find the biggest one
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)  # Calculate area of this shape
        print(f"   Object {i+1}: area = {area}")
        
        # Keep track of the largest object that meets our minimum size
        if area > max_area:
            max_area = area
            largest_contour = contour
    
    # STEP 8: Calculate center point and draw detection graphics
    if largest_contour is not None:
        print(f"✅ Best {color_name} object found! Area: {max_area}")
        
        # Calculate the center point using image moments
        # Moments give us statistical properties of the shape
        M = cv2.moments(largest_contour)
        
        if M["m00"] != 0:  # Make sure we can divide (avoid division by zero)
            # Calculate center coordinates within the detection area
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Adjust coordinates back to full frame if using detection area
            center = (cx + x, cy + y)
            
            # STEP 9: Draw detection visualizations on the original frame
            
            # Draw the contour outline in green
            adjusted_contour = largest_contour + [x, y]  # Adjust contour coordinates
            cv2.drawContours(original_frame, [adjusted_contour], -1, (0, 255, 0), 3)
            
            # Draw center point as blue circle
            cv2.circle(original_frame, center, 15, (255, 0, 0), -1)
            
            # Draw crosshairs at center point
            cv2.line(original_frame, (center[0]-20, center[1]), (center[0]+20, center[1]), (255, 255, 255), 2)
            cv2.line(original_frame, (center[0], center[1]-20), (center[0], center[1]+20), (255, 255, 255), 2)
            
            # Add text label above the detection
            label = f"{color_name.upper()} FOUND!"
            label_pos = (center[0] - 60, center[1] - 30)
            
            # Black background for text (better readability)
            cv2.rectangle(original_frame, (label_pos[0]-5, label_pos[1]-20), 
                         (label_pos[0]+120, label_pos[1]+5), (0, 0, 0), -1)
            # White text
            cv2.putText(original_frame, label, label_pos, 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Add area information
            area_text = f"Area: {int(max_area)}"
            cv2.putText(original_frame, area_text, (center[0] - 40, center[1] + 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        else:
            print(f"⚠️ Found {color_name} object but couldn't calculate center")
    else:
        print(f"❌ No {color_name} objects found meeting minimum area requirement")
    
    # Always return center (None if not found) and the annotated frame
    return center, original_frame

def detect_any_color(frame):
    """
    🌈 DETECT ANY COLOR FROM OUR SUPPORTED LIST
    
    This function tries to detect red, blue, green, or yellow objects in order.
    It returns the FIRST color it finds, so if there are multiple colors,
    it will pick based on the order: red → blue → green → yellow
    
    Parameters:
    - frame: Camera image to analyze
    
    Returns:
    - color_name: String name of detected color ("red", "blue", etc.) or None
    - center: (x, y) coordinates of detected object or None  
    - annotated_frame: Original frame with detection graphics added
    """
    
    colors_to_check = ["red", "blue", "green", "yellow"]  # Priority order
    
    print("🔍 Scanning for any colors...")
    
    # Try each color in order until we find one
    for color in colors_to_check:
        print(f"   Checking for {color}...")
        center, annotated_frame = detect_color(frame, color)
        
        if center is not None:  # Found this color!
            print(f"🎯 SUCCESS: Found {color} at position {center}")
            return color, center, annotated_frame
    
    # No colors found at all
    print("❌ No colors detected in frame")
    return None, None, frame

def react_to_detection(detected_color, center):
    """React to any color detection after 1 second delay - YES for correct, NO for wrong"""
    global last_detection_time, target_color
    current_time = time.time()
    
    # Cooldown to prevent spam
    if current_time - last_detection_time < detection_cooldown:
        return
    
    last_detection_time = current_time
    
    if not ARM_CONNECTED:
        print(f"🎯 {detected_color.upper()} detected - but no arm connected!")
        return
    
    if detected_color == target_color:
        # CORRECT COLOR! Nod yes and celebrate
        print(f"🎉 CORRECT! {detected_color.upper()} detected - Nodding YES and celebrating!")
        nod_yes()
        time.sleep(0.5)
        celebrate_dance()
    else:
        # WRONG COLOR! Shake no
        print(f"❌ WRONG! {detected_color.upper()} detected, looking for {target_color.upper()} - Shaking NO!")
        shake_no()

def draw_enhanced_display(frame):
    """Draw enhanced information on the camera frame"""
    global frame_count, fps, last_fps_time
    
    # Calculate FPS
    frame_count += 1
    current_time = time.time()
    if current_time - last_fps_time >= 1.0:
        fps = frame_count
        frame_count = 0
        last_fps_time = current_time
    
    # Get frame dimensions
    height, width = frame.shape[:2]
    
    # Create semi-transparent overlay for info
    overlay = frame.copy()
    
    # Top info bar
    cv2.rectangle(overlay, (0, 0), (width, 100), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
    
    # Bottom info bar
    cv2.rectangle(overlay, (0, height-60), (width, height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
    
    # System info
    timestamp = datetime.now().strftime("%H:%M:%S")
    cv2.putText(frame, f"DOFBOT Pro - {timestamp}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"FPS: {fps} | Resolution: {width}x{height}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    # Arm status
    arm_status = "🤖 CONNECTED" if ARM_CONNECTED else "⚠️ DISCONNECTED"
    cv2.putText(frame, f"Arm: {arm_status}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if ARM_CONNECTED else (0, 0, 255), 2)
    
    # Detection status
    if detection_mode == "active":
        status_text = f"🎯 Looking for: {target_color.upper()}"
        color = (0, 255, 0)
        
        # Show detection progress if detecting
        if hasattr(react_to_detection, 'current_color') and react_to_detection.current_color:
            if hasattr(react_to_detection, 'detection_start'):
                time_remaining = detection_delay - (time.time() - react_to_detection.detection_start)
                if time_remaining > 0:
                    progress_text = f"Detecting {react_to_detection.current_color.upper()}... {time_remaining:.1f}s"
                    cv2.putText(frame, progress_text, (10, height-35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    else:
        status_text = "🔍 Color detection: OFF"
        color = (128, 128, 128)
    
    cv2.putText(frame, status_text, (10, height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    # Controls help (right side)
    controls = [
        "ESC: Exit",
        "Commands in terminal:",
        "look_for_[color]",
        "test_servos",
        "snake_movement"
    ]
    
    for i, control in enumerate(controls):
        cv2.putText(frame, control, (width-200, 25 + i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    return frame

# === Handle Ctrl+C clean exit ===
def signal_handler(sig, frame_arg):
    print("\n[INFO] Exiting...")
    if ARM_CONNECTED:
        home()
    if 'cap' in globals() and cap.isOpened():
        cap.release()
        cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === Background thread for commands ===
def command_loop():
    global detection_mode, target_color
    while True:
        print("\n🤖 DOFBOT Commands:")
        print("🎯 Color Detection: look_for_red, look_for_blue, look_for_green, look_for_yellow, stop_looking")
        print("🎯 Detection Settings: full_screen_detection, center_detection, detection_info")
        print("🔧 Motor Tests: test_servos, test_speeds, test_gripper, snake_movement, robot_stretch")
        print("🎮 Actions: yarin_throw, cute_stand, grab, release, point, dance, shake_no, nod_yes, home")
        
        cmd = input("\n🧠 Enter command: ").strip().lower()
        
        if cmd == "yarin_throw":
            yarin_throw()
        elif cmd == "cute_stand":
            cute_stand()
        elif cmd == "grab":
            grab()
        elif cmd == "release":
            release()
        elif cmd == "point":
            point()
        elif cmd == "dance":
            dance()
        elif cmd == "celebrate_dance":
            celebrate_dance()
        elif cmd == "shake_no":
            shake_no()
        elif cmd == "nod_yes":
            nod_yes()
        elif cmd == "home":
            home()
        # Color detection commands
        elif cmd == "look_for_red":
            detection_mode = "active"
            target_color = "red"
            print("🔴 Looking for RED objects!")
            print(f"   Detection area: {DETECTION_AREA['width']}x{DETECTION_AREA['height']} pixels")
            print(f"   Detection time: {detection_delay} seconds")
            print("   Will nod YES + celebrate for red, shake NO for other colors!")
        elif cmd == "look_for_blue":
            detection_mode = "active"
            target_color = "blue"
            print("🔵 Looking for BLUE objects!")
            print(f"   Detection area: {DETECTION_AREA['width']}x{DETECTION_AREA['height']} pixels")
            print(f"   Detection time: {detection_delay} seconds")
            print("   Will nod YES + celebrate for blue, shake NO for other colors!")
        elif cmd == "look_for_green":
            detection_mode = "active"
            target_color = "green"
            print("🟢 Looking for GREEN objects!")
            print(f"   Detection area: {DETECTION_AREA['width']}x{DETECTION_AREA['height']} pixels")
            print(f"   Detection time: {detection_delay} seconds")
            print("   Will nod YES + celebrate for green, shake NO for other colors!")
        elif cmd == "look_for_yellow":
            detection_mode = "active"
            target_color = "yellow"
            print("🟡 Looking for YELLOW objects!")
            print(f"   Detection area: {DETECTION_AREA['width']}x{DETECTION_AREA['height']} pixels")
            print(f"   Detection time: {detection_delay} seconds")
            print("   Will nod YES + celebrate for yellow, shake NO for other colors!")
        elif cmd == "stop_looking":
            detection_mode = "none"
            target_color = "none"
            print("🚫 Stopped looking for colors!")
        # Detection area adjustment commands
        elif cmd == "full_screen_detection":
            DETECTION_AREA["enabled"] = False
            print("🖥️ Detection now uses FULL SCREEN!")
        elif cmd == "center_detection":
            DETECTION_AREA["enabled"] = True
            print(f"🎯 Detection now uses CENTER AREA: {DETECTION_AREA['width']}x{DETECTION_AREA['height']} pixels")
        elif cmd == "detection_info":
            print("\n📊 DETECTION SETTINGS:")
            print(f"   Area enabled: {DETECTION_AREA['enabled']}")
            print(f"   Detection zone: {DETECTION_AREA['width']}x{DETECTION_AREA['height']} pixels")
            print(f"   Zone position: ({DETECTION_AREA['x']}, {DETECTION_AREA['y']})")
            print(f"   Detection delay: {detection_delay} seconds")
            print(f"   Cooldown between reactions: {detection_cooldown} seconds")
        # Motor test commands
        elif cmd == "test_servos":
            test_individual_servos()
        elif cmd == "test_speeds":
            test_speed_variations()
        elif cmd == "test_gripper":
            test_gripper_range()
        elif cmd == "snake_movement":
            snake_movement()
        elif cmd == "robot_stretch":
            robot_stretch()
        elif cmd == "exit":
            signal_handler(None, None)
        else:
            print("❓ Unknown command! Check the list above.")

# === Start main logic ===
if __name__ == "__main__":
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    if not cap.isOpened():
        print("❌ Cannot open camera")
        sys.exit()

    print("🚀 [INFO] Starting enhanced DOFBOT Pro with rich display!")
    print("🎯 Enhanced camera display with FPS, timestamps, and system info")
    print("🔧 New motor test functions available")
    cv2.namedWindow("DOFBOT Pro Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("DOFBOT Pro Camera", 800, 600)

    if ARM_CONNECTED:
        home()

    # Start command loop in background
    threading.Thread(target=command_loop, daemon=True).start()

    # Main camera loop with enhanced display
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Camera frame failed.")
            continue

        # Perform color detection if mode is active
        if detection_mode == "active":
            try:
                # 🎯 MAIN DETECTION LOGIC WITH DETAILED COMMENTS
                
                print(f"🔍 [DETECTION FRAME] Looking for {target_color}...")
                
                # Try to find any color in the current frame
                detected_color, center, frame = detect_any_color(frame)
                
                if detected_color is not None and center is not None:
                    # 🎯 SOMETHING WAS DETECTED!
                    print(f"🎯 Detected: {detected_color} at {center}")
                    
                    current_time = time.time()
                    
                    # Check if this is a new detection or continuing from previous frames
                    if not hasattr(react_to_detection, 'current_color') or react_to_detection.current_color != detected_color:
                        # 🆕 NEW COLOR DETECTED - START TIMER
                        print(f"🆕 Starting {detection_delay}-second watch for {detected_color}")
                        react_to_detection.current_color = detected_color
                        react_to_detection.detection_start = current_time
                        react_to_detection.stored_center = center
                        
                        # Announce the detection
                        if detected_color == target_color:
                            print(f"🎉 TARGET COLOR {detected_color.upper()} spotted! Watching...")
                        else:
                            print(f"❌ Wrong color {detected_color.upper()} spotted (want {target_color}). Watching...")
                    
                    # Check if we've been seeing this same color for the full detection time
                    elif current_time - react_to_detection.detection_start >= detection_delay:
                        # ⏰ TIME'S UP - REACT TO THE DETECTED COLOR
                        print(f"⏰ {detection_delay} seconds elapsed! Reacting to {detected_color}")
                        react_to_detection(detected_color, react_to_detection.stored_center)
                        react_to_detection.current_color = None  # Reset for next detection
                    
                    else:
                        # ⏳ STILL WATCHING THE SAME COLOR
                        time_left = detection_delay - (current_time - react_to_detection.detection_start)
                        print(f"⏳ Still watching {detected_color}... {time_left:.1f}s remaining")
                
                else:
                    # ❌ NO COLOR DETECTED IN THIS FRAME
                    print("❌ No colors found in this frame")
                    
                    # Reset detection state since we lost the color
                    if hasattr(react_to_detection, 'current_color') and react_to_detection.current_color:
                        print(f"📴 Lost sight of {react_to_detection.current_color} - resetting timer")
                        react_to_detection.current_color = None
                        
            except Exception as e:
                # 🚨 ERROR HANDLING
                print(f"🚨 Detection error: {e}")
                print("🔄 Resetting detection state...")
                
                # Reset detection state on any error
                if hasattr(react_to_detection, 'current_color'):
                    react_to_detection.current_color = None
        
        # Add enhanced display
        frame = draw_enhanced_display(frame)

        cv2.imshow("DOFBOT Pro Camera", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            signal_handler(None, None)

import sys
import torch

# Add YOLOv5 to system path
sys.path.insert(0, './yolov5')

# Import from correct modules
from models.experimental import attempt_load
from utils.torch_utils import select_device

try:
    # Setup device
    device = select_device('')
    
    # Load model
    model = attempt_load('yolov5s.pt', map_location=device)
    model.eval()
    
    print("✅ YOLOv5 loaded successfully.")
except Exception as e:
    print(f"❌ Failed to load YOLOv5: {e}")


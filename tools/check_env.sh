#!/usr/bin/env bash

# Source environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/env.sh" ]; then
  source "$SCRIPT_DIR/env.sh"
fi

ALL_PASS=true

echo "=========================================="
echo "PS11-AUV Environment Verification"
echo "=========================================="
echo ""

# 1. DISPLAY & Xorg Session check
echo "--- [1/8] Display & Xorg Session Check ---"
echo "DISPLAY: $DISPLAY"
if [ -n "$XAUTHORITY" ]; then
  echo "XAUTHORITY: $XAUTHORITY"
fi

_USER_ID=$(id -u)
_SESSION_ID=$(loginctl list-sessions --no-legend 2>/dev/null | awk -v uid="$_USER_ID" '$2 == uid && $4 == "seat0" {print $1}' | head -n 1)
_SESSION_TYPE=""
if [ -n "$_SESSION_ID" ]; then
  _SESSION_TYPE=$(loginctl show-session "$_SESSION_ID" -p Type --value 2>/dev/null)
fi

echo "loginctl session ID: ${_SESSION_ID:-none}, Type: ${_SESSION_TYPE:-unknown}"
if [ "$_SESSION_TYPE" = "x11" ]; then
  echo "[PASS] Session type is x11 (Xorg)"
else
  echo "[FAIL] Session type is '$_SESSION_TYPE' (expected x11)"
  ALL_PASS=false
fi
echo ""

# 2. NVIDIA GPU & Driver check
echo "--- [2/8] NVIDIA GPU & Driver Check ---"
if command -v nvidia-smi &>/dev/null; then
  _GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n 1)
  _DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n 1)
  _TOTAL_MEM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null | head -n 1)
  echo "GPU: $_GPU_NAME"
  echo "Driver Version: $_DRIVER_VER"
  echo "Total Memory: $_TOTAL_MEM"
  if [ -n "$_GPU_NAME" ]; then
    echo "[PASS] nvidia-smi detected GPU ($_GPU_NAME)"
  else
    echo "[FAIL] nvidia-smi failed to query GPU"
    ALL_PASS=false
  fi
else
  echo "[FAIL] nvidia-smi not found"
  ALL_PASS=false
fi
echo ""

# 3. Gazebo Harmonic check
echo "--- [3/8] Gazebo Harmonic Check ---"
if command -v gz &>/dev/null; then
  _GZ_VER=$(gz sim --version 2>&1 | head -n 1)
  echo "Gazebo version output: $_GZ_VER"
  if echo "$_GZ_VER" | grep -q "version 8\."; then
    echo "[PASS] Gazebo Harmonic (8.x) detected"
  else
    echo "[FAIL] Gazebo version is not 8.x"
    ALL_PASS=false
  fi
else
  echo "[FAIL] gz command not found"
  ALL_PASS=false
fi
echo ""

# 4. ROS 2 Jazzy & Required Packages
echo "--- [4/8] ROS 2 Jazzy & Packages Check ---"
if [ "$ROS_DISTRO" = "jazzy" ]; then
  echo "[PASS] ROS 2 distro is jazzy"
else
  echo "[FAIL] ROS 2 distro is '${ROS_DISTRO}' (expected jazzy)"
  ALL_PASS=false
fi

_MISSING_PKGS=""
for pkg in ros_gz_bridge vision_msgs foxglove_bridge; do
  if ros2 pkg list 2>/dev/null | grep -qx "$pkg"; then
    echo "  - Package '$pkg': installed"
  else
    echo "  - Package '$pkg': MISSING"
    _MISSING_PKGS="$_MISSING_PKGS $pkg"
  fi
done

if [ -z "$_MISSING_PKGS" ]; then
  echo "[PASS] All required ROS 2 packages present"
else
  echo "[FAIL] Missing ROS 2 packages:$_MISSING_PKGS"
  ALL_PASS=false
fi
echo ""

# 5. Python Virtual Environment Check
echo "--- [5/8] Python Virtual Environment Check ---"
_PYTHON_PATH=$(which python3)
echo "Python path: $_PYTHON_PATH"
if [[ "$_PYTHON_PATH" == *"/venvs/ps11/"* ]]; then
  echo "[PASS] Active Python is inside ~/venvs/ps11"
else
  echo "[FAIL] Python is not running from ~/venvs/ps11"
  ALL_PASS=false
fi
echo ""

# 6. PyTorch & CUDA Check
echo "--- [6/8] PyTorch CUDA Check ---"
python3 -c "
import sys
try:
    import torch
    print(f'PyTorch version: {torch.__version__}')
    cuda_avail = torch.cuda.is_available()
    print(f'CUDA available: {cuda_avail}')
    if cuda_avail:
        print(f'Device name: {torch.cuda.get_device_name(0)}')
        print(f'Device count: {torch.cuda.device_count()}')
        sys.exit(0)
    else:
        sys.exit(1)
except Exception as e:
    print(f'Error: {e}')
    sys.exit(1)
"
if [ $? -eq 0 ]; then
  echo "[PASS] PyTorch detects CUDA GPU successfully"
else
  echo "[FAIL] PyTorch CUDA check failed"
  ALL_PASS=false
fi
echo ""

# 7. NumPy & ROS Compatibility Check
echo "--- [7/8] NumPy Version & rclpy Compatibility Check ---"
python3 -c "
import sys
import numpy as np
print(f'NumPy version: {np.__version__}')

# Test rclpy import and basic message serialization
try:
    import rclpy
    import ultralytics
    import supervision
    print('Imports: rclpy, ultralytics, supervision OK')
except Exception as e:
    print(f'Import error: {e}')
    sys.exit(1)

# Check numpy version compatibility rule (< 2.0.0 for ROS Jazzy built against numpy 1.26)
major = int(np.__version__.split('.')[0])
if major >= 2:
    print(f'WARNING: NumPy {np.__version__} >= 2.0.0! ROS Jazzy requires numpy<2.0.0.')
    sys.exit(1)
sys.exit(0)
"
if [ $? -eq 0 ]; then
  echo "[PASS] NumPy version conforms to ROS 2 Jazzy requirements (<2.0)"
else
  echo "[FAIL] NumPy compatibility check failed"
  ALL_PASS=false
fi
# 8. Node Executable Shebang Check
echo "--- [8/8] Node Executable Shebang Check ---"
_EMULATOR_SCRIPT="${PS11_ROOT:-$SCRIPT_DIR/..}/ros2_ws/install/ps11_telemetry/lib/ps11_telemetry/link_emulator"
if [ -f "$_EMULATOR_SCRIPT" ]; then
  _SHEBANG=$(head -n 1 "$_EMULATOR_SCRIPT")
  echo "link_emulator shebang: $_SHEBANG"
  if [[ "$_SHEBANG" == *"venvs/ps11/bin/python"* ]] || [[ "$_SHEBANG" == "#!/usr/bin/env python3"* ]]; then
    echo "[PASS] Installed node script uses venv Python interpreter"
  else
    echo "[FAIL] Installed node script uses unexpected interpreter ($_SHEBANG)"
    ALL_PASS=false
  fi
else
  echo "[FAIL] $_EMULATOR_SCRIPT not found. Build workspace first."
  ALL_PASS=false
fi
echo ""

# Summary
echo "=========================================="
if [ "$ALL_PASS" = true ]; then
  echo "OVERALL: ALL CHECKS PASSED"
  echo "=========================================="
  exit 0
else
  echo "OVERALL: ONE OR MORE CHECKS FAILED"
  echo "=========================================="
  exit 1
fi

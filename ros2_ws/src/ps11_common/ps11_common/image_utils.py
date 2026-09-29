"""Pure-NumPy image conversion utilities for ROS 2 without cv_bridge."""

import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

SUPPORTED_ENCODINGS = ("rgb8", "bgr8", "32FC1")


def image_to_numpy(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image message to a numpy ndarray.

    Supported encodings:
    - 'rgb8': returns (H, W, 3) uint8 array in RGB order
    - 'bgr8': returns (H, W, 3) uint8 array in BGR order
    - '32FC1': returns (H, W) float32 array
    """
    if msg.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(
            f"Unsupported encoding '{msg.encoding}'. Supported: {SUPPORTED_ENCODINGS}"
        )

    if msg.encoding in ("rgb8", "bgr8"):
        dtype = np.uint8
        channels = 3
        expected_bytes_per_row = msg.width * channels * dtype().itemsize
        if msg.step < expected_bytes_per_row:
            raise ValueError(
                f"Image step ({msg.step}) is smaller than row bytes ({expected_bytes_per_row})"
            )
        # Handle possible row padding
        raw = np.frombuffer(msg.data, dtype=dtype)
        if msg.step == expected_bytes_per_row:
            arr = raw.reshape((msg.height, msg.width, channels))
        else:
            # Row padded
            arr = raw.reshape((msg.height, msg.step))[:, :expected_bytes_per_row]
            arr = arr.reshape((msg.height, msg.width, channels))
        return arr.copy()

    elif msg.encoding == "32FC1":
        dtype = np.float32
        expected_bytes_per_row = msg.width * dtype().itemsize
        if msg.step < expected_bytes_per_row:
            raise ValueError(
                f"Image step ({msg.step}) is smaller than row bytes ({expected_bytes_per_row})"
            )
        raw = np.frombuffer(msg.data, dtype=dtype)
        if msg.step == expected_bytes_per_row:
            arr = raw.reshape((msg.height, msg.width))
        else:
            row_elements = msg.step // dtype().itemsize
            arr = raw.reshape((msg.height, row_elements))[:, : msg.width]
        return arr.copy()

    raise ValueError(f"Unhandled encoding: {msg.encoding}")


def numpy_to_image(
    arr: np.ndarray,
    encoding: str,
    header: Header | None = None,
) -> Image:
    """Convert a numpy ndarray to a sensor_msgs/Image message.

    Supported encodings:
    - 'rgb8': requires (H, W, 3) uint8 array
    - 'bgr8': requires (H, W, 3) uint8 array
    - '32FC1': requires (H, W) or (H, W, 1) float32 array
    """
    if encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(
            f"Unsupported encoding '{encoding}'. Supported: {SUPPORTED_ENCODINGS}"
        )

    if encoding in ("rgb8", "bgr8"):
        if arr.dtype != np.uint8:
            raise ValueError(
                f"Array dtype for '{encoding}' must be uint8, got {arr.dtype}"
            )
        if arr.ndim != 3 or arr.shape[2] != 3:
            raise ValueError(
                f"Array shape for '{encoding}' must be (H, W, 3), got {arr.shape}"
            )
        height, width, _channels = arr.shape
        step = width * 3

    elif encoding == "32FC1":
        if arr.dtype != np.float32:
            raise ValueError(
                f"Array dtype for '32FC1' must be float32, got {arr.dtype}"
            )
        if arr.ndim == 2:
            height, width = arr.shape
        elif arr.ndim == 3 and arr.shape[2] == 1:
            height, width = arr.shape[0], arr.shape[1]
            arr = arr.squeeze(axis=2)
        else:
            raise ValueError(
                f"Array shape for '32FC1' must be (H, W) or (H, W, 1), got {arr.shape}"
            )
        step = width * 4

    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(step)
    msg.data = arr.tobytes()
    return msg


def numpy_to_compressed_image(
    arr: np.ndarray,
    quality: int = 80,
    header: Header | None = None,
    encoding: str = "rgb8",
) -> CompressedImage:
    """Convert numpy array (RGB or BGR) to sensor_msgs/CompressedImage (JPEG)."""
    import cv2

    if encoding == "rgb8":
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    elif encoding == "bgr8":
        bgr = arr
    else:
        raise ValueError(
            f"Unsupported encoding '{encoding}' for JPEG compression. Use 'rgb8' or 'bgr8'."
        )

    success, enc = cv2.imencode(
        ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not success:
        raise RuntimeError("cv2.imencode failed to compress image to JPEG")

    msg = CompressedImage()
    if header is not None:
        msg.header = header
    msg.format = "jpeg"
    msg.data = enc.tobytes()
    return msg


def compressed_image_to_numpy(
    msg: CompressedImage,
    target_encoding: str = "rgb8",
) -> np.ndarray:
    """Decode a sensor_msgs/CompressedImage (JPEG) to a numpy ndarray."""
    import cv2

    np_arr = np.frombuffer(msg.data, np.uint8)
    bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("cv2.imdecode failed to decode CompressedImage data")

    if target_encoding == "rgb8":
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    elif target_encoding == "bgr8":
        return bgr
    else:
        raise ValueError(
            f"Unsupported target_encoding '{target_encoding}'. Use 'rgb8' or 'bgr8'."
        )


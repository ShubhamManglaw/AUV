"""Tests for pure-NumPy ROS image conversion utilities."""

import numpy as np
import pytest
from ps11_common.image_utils import image_to_numpy, numpy_to_image
from sensor_msgs.msg import Image


def test_roundtrip_rgb8() -> None:
    h, w = 480, 640
    original = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)

    msg = numpy_to_image(original, encoding="rgb8")
    assert msg.height == h
    assert msg.width == w
    assert msg.encoding == "rgb8"
    assert msg.step == w * 3

    recovered = image_to_numpy(msg)
    assert recovered.shape == (h, w, 3)
    assert recovered.dtype == np.uint8
    np.testing.assert_array_equal(recovered, original)


def test_roundtrip_bgr8() -> None:
    h, w = 120, 160
    original = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)

    msg = numpy_to_image(original, encoding="bgr8")
    assert msg.height == h
    assert msg.width == w
    assert msg.encoding == "bgr8"

    recovered = image_to_numpy(msg)
    assert recovered.shape == (h, w, 3)
    assert recovered.dtype == np.uint8
    np.testing.assert_array_equal(recovered, original)


def test_roundtrip_32fc1() -> None:
    h, w = 240, 320
    original = np.random.uniform(0.5, 25.0, (h, w)).astype(np.float32)

    msg = numpy_to_image(original, encoding="32FC1")
    assert msg.height == h
    assert msg.width == w
    assert msg.encoding == "32FC1"
    assert msg.step == w * 4

    recovered = image_to_numpy(msg)
    assert recovered.shape == (h, w)
    assert recovered.dtype == np.float32
    np.testing.assert_array_almost_equal(recovered, original, decimal=6)


def test_unsupported_encoding() -> None:
    arr = np.zeros((10, 10), dtype=np.uint8)
    with pytest.raises(ValueError, match="Unsupported encoding"):
        numpy_to_image(arr, encoding="mono8")

    dummy_msg = Image()
    dummy_msg.encoding = "invalid_enc"
    with pytest.raises(ValueError, match="Unsupported encoding"):
        image_to_numpy(dummy_msg)


def test_mismatched_dtype_or_shape() -> None:
    # Float array with rgb8
    arr_float = np.zeros((10, 10, 3), dtype=np.float32)
    with pytest.raises(ValueError, match="must be uint8"):
        numpy_to_image(arr_float, encoding="rgb8")

    # 2D array with rgb8
    arr_2d = np.zeros((10, 10), dtype=np.uint8)
    with pytest.raises(ValueError, match="must be \\(H, W, 3\\)"):
        numpy_to_image(arr_2d, encoding="rgb8")

    # uint8 array with 32FC1
    arr_uint8 = np.zeros((10, 10), dtype=np.uint8)
    with pytest.raises(ValueError, match="must be float32"):
        numpy_to_image(arr_uint8, encoding="32FC1")

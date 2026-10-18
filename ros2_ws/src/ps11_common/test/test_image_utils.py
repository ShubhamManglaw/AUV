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


def test_roundtrip_compressed_image_rgb() -> None:
    from ps11_common.image_utils import (
        compressed_image_to_numpy,
        numpy_to_compressed_image,
    )

    h, w = 240, 320
    # Generate smooth gradient image so JPEG compression doesn't destroy it
    x = np.linspace(0, 255, w, dtype=np.uint8)
    y = np.linspace(0, 255, h, dtype=np.uint8)
    xx, yy = np.meshgrid(x, y)
    original = np.stack([xx, yy, (xx // 2 + yy // 2)], axis=-1)

    comp_msg = numpy_to_compressed_image(original, quality=95, encoding="rgb8")
    assert comp_msg.format == "jpeg"
    assert len(comp_msg.data) > 0

    recovered = compressed_image_to_numpy(comp_msg, target_encoding="rgb8")
    assert recovered.shape == original.shape
    # Lossy JPEG will have small differences, verify mean error is small (< 5 px)
    mean_err = np.mean(np.abs(original.astype(float) - recovered.astype(float)))
    assert mean_err < 5.0


def test_roundtrip_compressed_image_bgr() -> None:
    from ps11_common.image_utils import (
        compressed_image_to_numpy,
        numpy_to_compressed_image,
    )

    h, w = 100, 100
    original = np.full((h, w, 3), 128, dtype=np.uint8)

    comp_msg = numpy_to_compressed_image(original, quality=80, encoding="bgr8")
    recovered = compressed_image_to_numpy(comp_msg, target_encoding="bgr8")
    assert recovered.shape == original.shape
    np.testing.assert_allclose(recovered, original, atol=2.0)


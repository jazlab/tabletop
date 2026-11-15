from array import array

import numpy as np
import pytest
from sensor_msgs.msg import Image
from tabletop_rig.nodes.camera_grid import CAMERAS, preview_frame, render


def test_preview_stride_padding_and_owned_buffer():
    message = Image(
        height=2,
        width=2,
        encoding="mono8",
        step=3,
        data=array("B", [10, 20, 255, 30, 40, 255]),
    )
    result = preview_frame(message)
    del message
    assert result.tolist() == [[10, 20], [30, 40]]


def test_rgb_preview_converts_to_bgr():
    message = Image(
        height=1,
        width=1,
        encoding="rgb8",
        step=3,
        data=array("B", [255, 0, 0]),
    )
    assert preview_frame(message).tolist() == [[[0, 0, 255]]]


@pytest.mark.parametrize(
    "encoding,step,data",
    [
        ("mono16", 2, [0, 0]),
        ("mono8", 0, [0]),
        ("mono8", 2, [0]),
    ],
)
def test_invalid_preview_is_rejected(encoding, step, data):
    with pytest.raises(ValueError):
        preview_frame(
            Image(
                height=1,
                width=2,
                encoding=encoding,
                step=step,
                data=array("B", data),
            )
        )


def test_grid_has_six_distinct_preview_cells_and_handles_stale():
    frames = {
        camera: (np.full((40, 64), 30 + i * 30, np.uint8), 10.0)
        for i, (camera, _) in enumerate(CAMERAS)
    }
    canvas = render(frames, width=1000, height=600, now=11.0)
    assert canvas.shape == (600, 1000, 3)
    for value in (30, 60, 90, 120, 150, 180):
        assert np.any(np.all(canvas == value, axis=2))
    assert render({}, width=1000, height=600).shape == canvas.shape
    assert (
        render(frames, width=1000, height=600, now=20.0).shape == canvas.shape
    )

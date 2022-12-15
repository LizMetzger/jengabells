"""Test converting euler coordinates to quaternians."""
import numpy as np
from camera.calibrate import quaternion_multiply


def test_identity():
    assert np.allclose(quaternion_multiply([0.707, 0.0, 0.0, 0.707],
                                           [0.0, 0.707, 0.707, 0.0]), [0, 0, 0.999698, 0])

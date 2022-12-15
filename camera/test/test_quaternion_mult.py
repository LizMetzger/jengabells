"""Test converting euler coordinates to quaternians."""
import numpy as np
from camera.calibrate import quaternion_from_euler


def test_identity():
    assert np.allclose(quaternion_from_euler(1, .5, 0),
                                            (0.46452136, 0.2171174, -0.11861178, 0.85030065))

"""Tests of the pure functions in rapid_nav_tools.checks, with hand-computed values."""

import math

import numpy as np
import pytest

from rapid_nav_tools.checks import (
    angle_diff,
    colourise_depth,
    expected_fx,
    planar_delta,
    quaternion_from_rpy,
    rate_verdict,
    scan_image,
    yaw_from_quaternion,
)


def test_expected_fx_matches_the_simulated_camera():
    """Return 462.27 pixels for a 640 px camera with 1.211 rad, as the simulated camera_info."""
    assert expected_fx(640, 1.211) == pytest.approx(462.27, abs=0.02)


def test_expected_fx_halves_with_half_the_width_at_the_same_field_of_view():
    """Make focal length in pixels proportional to the width at one field of view."""
    assert expected_fx(320, 1.211) == pytest.approx(expected_fx(640, 1.211) / 2.0)


def test_quaternion_of_the_optical_frame_rotation():
    """Return (-0.5, 0.5, -0.5, 0.5) for rpy (-pi/2, 0, -pi/2)."""
    quaternion = quaternion_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
    assert quaternion == pytest.approx((-0.5, 0.5, -0.5, 0.5), abs=1e-12)


def test_quaternion_identity_and_pure_yaw():
    """A zero rotation is the identity, and a yaw of 90 degrees is about the z axis."""
    assert quaternion_from_rpy(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


def test_yaw_round_trip():
    """yaw_from_quaternion inverts quaternion_from_rpy for a pure yaw."""
    for yaw in (-2.5, -0.3, 0.0, 0.7, 3.0):
        quaternion = quaternion_from_rpy(0.0, 0.0, yaw)
        assert yaw_from_quaternion(*quaternion) == pytest.approx(yaw, abs=1e-9)


def test_angle_diff_wraps_across_pi():
    """Differences across the +/- pi boundary take the short way round."""
    assert angle_diff(3.1, -3.1) == pytest.approx(-0.0831853, abs=1e-6)
    assert angle_diff(-3.1, 3.1) == pytest.approx(0.0831853, abs=1e-6)


def test_angle_diff_interval_is_half_open():
    """The result lies in (-pi, pi]: pi stays pi, and a plain difference is unchanged."""
    assert angle_diff(math.pi, 0.0) == pytest.approx(math.pi)
    assert angle_diff(0.1, 0.0) == pytest.approx(0.1)
    assert angle_diff(0.0, 0.1) == pytest.approx(-0.1)


def test_rate_verdict_native():
    """Native: PASS at 80 % of the expected rate, WARN below it, FAIL with no messages."""
    assert rate_verdict(300, 29.9, 30, 'native') == 'PASS'
    assert rate_verdict(240, 24.0, 30, 'native') == 'PASS'
    assert rate_verdict(200, 20.0, 30, 'native') == 'WARN'
    assert rate_verdict(0, 0.0, 30, 'native') == 'FAIL'


def test_rate_verdict_docker_only_checks_presence():
    """Docker: any message passes, no message fails, whatever the rate."""
    assert rate_verdict(5, 0.5, 30, 'docker') == 'PASS'
    assert rate_verdict(0, 0.0, 30, 'docker') == 'FAIL'


def test_rate_verdict_rejects_an_unknown_profile():
    """An unknown profile name is an error, not a silent PASS."""
    with pytest.raises(ValueError):
        rate_verdict(10, 10.0, 10, 'gpu')


def test_colourise_depth_colours_and_non_finite():
    """Near is blue, far is red, the middle is green-ish, and non-finite values are black."""
    depth = np.array([[0.1, 10.0], [np.nan, 5.05]])
    rgb = colourise_depth(depth, 0.1, 10.0)
    assert rgb.shape == (2, 2, 3)
    assert rgb.dtype == np.uint8
    assert tuple(rgb[0, 0]) == (0, 0, 255)
    assert tuple(rgb[0, 1]) == (255, 0, 0)
    assert tuple(rgb[1, 0]) == (0, 0, 0)
    assert rgb[1, 1, 1] > 200


def test_scan_image_places_a_return_ahead_of_the_robot():
    """
    A return 1 m ahead at max range 2 m lands 25 px right of the centre on a 101 px view.

    Infinite and NaN returns are ignored, so the only white pixel is the return and the only
    red pixel is the robot.
    """
    ranges = np.array([1.0, np.inf, np.nan])
    image = scan_image(ranges, 0.0, 0.0, 101, 2.0)
    assert image.shape == (101, 101, 3)
    assert tuple(image[50, 75]) == (255, 255, 255)
    assert tuple(image[50, 50]) == (255, 0, 0)
    white = np.all(image == 255, axis=-1)
    assert int(white.sum()) == 1


def test_scan_image_maps_left_to_up():
    """A return at +90 degrees (left of the robot, y positive) appears above the centre."""
    image = scan_image(np.array([1.0]), math.pi / 2, 0.0, 101, 2.0)
    assert tuple(image[25, 50]) == (255, 255, 255)


def test_planar_delta_distance_and_wrapped_yaw():
    """The distance is the straight-line length; the yaw change is wrapped."""
    distance, yaw_change = planar_delta((0.0, 0.0, 0.0), (1.0, 0.0, 0.5))
    assert distance == pytest.approx(1.0)
    assert yaw_change == pytest.approx(0.5)
    distance, yaw_change = planar_delta((0.0, 0.0, 3.1), (0.0, 0.0, -3.1))
    assert distance == pytest.approx(0.0)
    assert yaw_change == pytest.approx(0.0831853, abs=1e-6)
    distance, _ = planar_delta((1.0, 1.0, 0.0), (4.0, 5.0, 0.0))
    assert distance == pytest.approx(5.0)

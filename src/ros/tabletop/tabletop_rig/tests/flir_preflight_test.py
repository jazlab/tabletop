from tabletop_rig.utils.flir_preflight import CAMERAS, evaluate_preflight


def test_preflight_passes_with_images_calibration_and_full_rates():
    images = set(CAMERAS)
    calibration = {camera: {} for camera in CAMERAS}
    rates = {camera: 119.8 for camera in CAMERAS}

    status, failures = evaluate_preflight(images, calibration, rates, 115.0)

    assert status == "PASS"
    assert failures == []


def test_preflight_reports_each_missing_requirement():
    missing = CAMERAS[0]
    images = set(CAMERAS) - {missing}
    calibration = {camera: {} for camera in CAMERAS if camera != missing}
    rates = {camera: 120.0 for camera in CAMERAS}
    rates[missing] = 90.0

    status, failures = evaluate_preflight(images, calibration, rates, 115.0)

    assert status == "FAIL"
    assert any("no compressed image" in reason for reason in failures)
    assert any("no CameraInfo" in reason for reason in failures)
    assert any("90.0 Hz" in reason for reason in failures)

"""Unit tests for gaze estimation geometric models.

This module contains pytest test cases for the gaze estimation model
components in tabletop_py.gaze.models, including:

- nearest_focus: Finding closest points between two 3D lines
- intersect_ray_sphere: Ray-sphere intersection for eye modeling
- GazeEstimationModelGeometric: Full geometric gaze estimation model

Test Classes:
    TestNearestFocus: Tests for line-line closest point algorithms.
    TestIntersectRaySphere: Tests for ray-sphere intersection.
    TestGazeEstimationModelGeometric: Integration tests for full model.

Helper Functions:
    assert_tensors_equal: Compare tensors with tolerance.
    assert_all_tensors_equal: Compare lists of tensors.
    get_default_model_params: Create default model initialization parameters.

Example:
    pytest tests/gaze_estimation_test.py -v
"""

from typing import Any

import pytest
import torch

from tabletop_py.gaze.models import (
    GazeEstimationModelGeometric,
    LearnableMaskedCorrectionParameter,
    decompose_tf,
    intersect_ray_sphere,
    nearest_focus,
    nearest_focus_cross,
)


def assert_tensors_equal(
    t0: torch.Tensor, t1: torch.Tensor, tol: float = 1e-6
):
    """Assert two tensors are approximately equal.

    Args:
        t0: First tensor to compare.
        t1: Second tensor to compare.
        tol: Absolute tolerance for comparison (default: 1e-6).

    Raises:
        AssertionError: If tensors differ by more than tolerance.
    """
    assert torch.allclose(t0, t1, atol=tol), f"Tensors not equal:\n{t0}\n{t1}"


def assert_all_tensors_equal(
    t0s: list[torch.Tensor] | tuple[torch.Tensor, ...],
    t1s: list[torch.Tensor] | tuple[torch.Tensor, ...],
    tol: float = 1e-6,
):
    """Assert corresponding tensors in two sequences are approximately equal.

    Args:
        t0s: First sequence of tensors.
        t1s: Second sequence of tensors.
        tol: Absolute tolerance for comparison (default: 1e-6).

    Raises:
        AssertionError: If any corresponding tensor pair differs.
    """
    for t0, t1 in zip(t0s, t1s):
        assert_tensors_equal(t0, t1, tol)


class TestNearestFocus:
    """Test cases for nearest_focus line intersection algorithms.

    Tests the nearest_focus and nearest_focus_cross functions which find
    the closest points between two 3D lines (rays). These are used in
    gaze estimation to find where left and right eye gaze vectors converge.
    """

    def test_simple_non_parallel_lines(self):
        """Test basic case with perpendicular, non-intersecting lines."""
        p0 = torch.tensor([[0.0, 0.0, 0.0]])
        v0 = torch.tensor([[1.0, 0.0, 0.0]])
        p1 = torch.tensor([[0.0, 1.0, 0.0]])
        v1 = torch.tensor([[0.0, 0.0, 1.0]])

        res = nearest_focus(p0, v0, p1, v1)
        res_cross = nearest_focus_cross(p0, v0, p1, v1)

        assert_all_tensors_equal(res, res_cross)

    def test_intersecting_lines(self):
        p0 = torch.tensor([[0.0, 0.0, 0.0]])
        v0 = torch.tensor([[1.0, 0.0, 0.0]])
        p1 = torch.tensor([[0.0, 0.0, 0.0]])
        v1 = torch.tensor([[0.0, 1.0, 0.0]])

        q_mid, q0, q1, distance = nearest_focus(p0, v0, p1, v1)
        q_mid_cross, q0_cross, q1_cross, distance_cross = nearest_focus_cross(
            p0, v0, p1, v1
        )
        assert_tensors_equal(q_mid, q_mid_cross)
        assert_tensors_equal(q0, q0_cross)
        assert_tensors_equal(q1, q1_cross)
        assert_tensors_equal(distance, distance_cross)

    def test_parallel_lines(self):
        p0 = torch.tensor([[0.0, 0.0, 0.0]])
        v0 = torch.tensor([[1.0, 0.0, 0.0]])
        p1 = torch.tensor([[0.0, 1.0, 0.0]])
        v1 = torch.tensor([[1.0, 0.0, 0.0]])  # Parallel to v0

        with pytest.raises(ValueError, match="Parallel lines detected"):
            nearest_focus(p0, v0, p1, v1)

        with pytest.raises(ValueError, match="Parallel lines detected"):
            nearest_focus_cross(p0, v0, p1, v1)

    def test_batch_processing(self):
        p0 = torch.tensor([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
        v0 = torch.tensor([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        p1 = torch.tensor([[0.0, 1.0, 0.0], [2.0, 0.0, 1.0]])
        v1 = torch.tensor([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])

        q_mid, q0, q1, distance = nearest_focus(p0, v0, p1, v1)
        q_mid_cross, q0_cross, q1_cross, distance_cross = nearest_focus_cross(
            p0, v0, p1, v1
        )
        assert_tensors_equal(q_mid, q_mid_cross)
        assert_tensors_equal(q0, q0_cross)
        assert_tensors_equal(q1, q1_cross)
        assert_tensors_equal(distance, distance_cross)

    def test_skew_lines(self):
        p0 = torch.tensor([[0.0, 0.0, 0.0]])
        v0 = torch.tensor([[1.0, 1.0, 1.0]])
        p1 = torch.tensor([[1.0, 0.0, 0.0]])
        v1 = torch.tensor([[0.0, 1.0, -1.0]])

        q_mid, q0, q1, distance = nearest_focus(p0, v0, p1, v1)
        q_mid_cross, q0_cross, q1_cross, distance_cross = nearest_focus_cross(
            p0, v0, p1, v1
        )
        assert_tensors_equal(q_mid, q_mid_cross)
        assert_tensors_equal(q0, q0_cross)
        assert_tensors_equal(q1, q1_cross)
        assert_tensors_equal(distance, distance_cross)

    def test_almost_parallel_lines(self):
        p0 = torch.tensor([[0.0, 0.0, 0.0]])
        v0 = torch.tensor([[1.0, 0.0, 0.0]])
        p1 = torch.tensor([[0.0, 1.0, 0.0]])
        v1 = torch.tensor([[1.0, 1e-9, 0.0]])  # Almost parallel to v0

        with pytest.raises(ValueError, match="Parallel lines detected"):
            nearest_focus(p0, v0, p1, v1)
        with pytest.raises(ValueError, match="Parallel lines detected"):
            nearest_focus_cross(p0, v0, p1, v1)

    # def test_parallel_lines_with_custom_epsilon(self):
    #     p0 = torch.tensor([[0.0, 0.0, 0.0]])
    #     v0 = torch.tensor([[1.0, 0.0, 0.0]])
    #     p1 = torch.tensor([[0.0, 1.0, 0.0]])
    #     # Make v1 very slightly different from v0
    #     # v0 dot v1 = 1 - (1e-4)^2,  v0v1^2 = (1 - (1e-4)^2)^2 approx 1 - 2*(1e-4)^2
    #     # det = v0v1^2 - 1 = approx -2e-8. Default epsilon is 1e-7.
    #     # So this should be considered parallel if epsilon is larger, e.g., 1e-6
    #     # And not parallel if epsilon is smaller, e.g., 1e-9
    #     v1_almost_parallel = torch.tensor([[1.0 - 1e-7, 1e-4, 0.0]])

    #     # Test with epsilon that should detect parallelism
    #     with pytest.raises(ValueError, match="Parallel lines detected"):
    #         nearest_focus(
    #             p0, v0, p1, v1_almost_parallel, epsilon=1e-7
    #         )  # default epsilon
    #     with pytest.raises(ValueError, match="Parallel lines detected"):
    #         nearest_focus_cross(
    #             p0, v0, p1, v1_almost_parallel, epsilon=1e-7
    #         )  # default epsilon

    #     # Test with epsilon that should NOT detect parallelism
    #     try:
    #         nearest_focus(p0, v0, p1, v1_almost_parallel, epsilon=1e-9)
    #     except ValueError as e:
    #         pytest.fail(
    #             f"nearest_focus raised ValueError with small epsilon: {e}"
    #         )
    #     try:
    #         nearest_focus_cross(p0, v0, p1, v1_almost_parallel, epsilon=1e-9)
    #     except ValueError as e:
    #         pytest.fail(
    #             f"nearest_focus_cross raised ValueError with small epsilon: {e}"
    #         )


class TestIntersectRaySphere:
    """Test cases for ray-sphere intersection algorithm.

    Tests the intersect_ray_sphere function which computes where a ray
    intersects a sphere surface. This is used to model how gaze rays
    exit the eyeball (approximated as a sphere).
    """

    def test_single_intersection(self):
        """Test ray hitting sphere center from outside."""
        ray_origin = torch.tensor([[0.0, 0.0, -2.0]])
        ray_direction = torch.tensor([[0.0, 0.0, 1.0]])
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])

        expected_intersection = torch.tensor([[0.0, 0.0, -1.0]])
        intersection = intersect_ray_sphere(
            ray_origin, ray_direction, sphere_center, sphere_radius
        )
        assert_tensors_equal(intersection, expected_intersection)

    def test_batch_intersection(self):
        ray_origin = torch.tensor([[0.0, 0.0, -2.0], [0.0, 0.0, -3.0]])
        ray_direction = torch.tensor([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
        sphere_center = torch.tensor([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0, 2.0])

        expected_intersection = torch.tensor(
            [[0.0, 0.0, -1.0], [0.0, 0.0, -2.0]]
        )
        intersection = intersect_ray_sphere(
            ray_origin,
            ray_direction,
            sphere_center,
            sphere_radius,
        )
        assert_tensors_equal(intersection, expected_intersection)

    def test_tangent_intersection(self):
        ray_origin = torch.tensor([[1.0, 0.0, -2.0]])
        ray_direction = torch.tensor([[0.0, 0.0, 1.0]])
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])

        expected_intersection = torch.tensor([[1.0, 0.0, 0.0]])
        intersection = intersect_ray_sphere(
            ray_origin, ray_direction, sphere_center, sphere_radius
        )
        assert_tensors_equal(intersection, expected_intersection, tol=1e-5)

    def test_ray_misses_sphere(self):
        ray_origin = torch.tensor([[2.0, 0.0, -2.0]])  # Shifted to miss
        ray_direction = torch.tensor([[0.0, 0.0, 1.0]])
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])

        with pytest.raises(RuntimeError, match="Ray misses the sphere"):
            intersect_ray_sphere(
                ray_origin, ray_direction, sphere_center, sphere_radius
            )

    def test_ray_origin_inside_sphere(self):
        ray_origin = torch.tensor([[0.0, 0.0, 0.0]])  # Origin inside sphere
        ray_direction = torch.tensor([[0.0, 0.0, 1.0]])
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])

        with pytest.raises(
            AssertionError, match="Ray origin is inside the sphere"
        ):
            intersect_ray_sphere(
                ray_origin, ray_direction, sphere_center, sphere_radius
            )

    def test_ray_pointing_away_from_sphere(self):
        ray_origin = torch.tensor([[0.0, 0.0, -2.0]])
        ray_direction = torch.tensor([[0.0, 0.0, -1.0]])  # Pointing away
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])

        with pytest.raises(
            AssertionError, match="Ray is pointing away from the sphere"
        ):
            intersect_ray_sphere(
                ray_origin, ray_direction, sphere_center, sphere_radius
            )

    def test_intersection_at_sphere_surface_origin_outside(self):
        # Ray origin is outside, but very close, and points directly at center
        ray_origin = torch.tensor([[0.0, 0.0, -1.0 - 1e-7]])
        ray_direction = torch.tensor([[0.0, 0.0, 1.0]])
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])
        expected_intersection = torch.tensor([[0.0, 0.0, -1.0]])
        intersection = intersect_ray_sphere(
            ray_origin, ray_direction, sphere_center, sphere_radius
        )
        assert_tensors_equal(intersection, expected_intersection, tol=1e-6)

    def test_ray_origin_on_sphere_surface_pointing_inwards(self):
        # Ray origin exactly on the sphere surface, pointing inwards.
        # This should fail the c > 0 assertion (ray origin inside sphere)
        # as c = dot(d, d) - r*r will be 0.
        ray_origin = torch.tensor([[0.0, 0.0, -1.0]])
        ray_direction = torch.tensor([[0.0, 0.0, 1.0]])  # Pointing inwards
        sphere_center = torch.tensor([[0.0, 0.0, 0.0]])
        sphere_radius = torch.tensor([1.0])
        with pytest.raises(
            AssertionError, match="Ray origin is inside the sphere"
        ):
            intersect_ray_sphere(
                ray_origin, ray_direction, sphere_center, sphere_radius
            )


def get_default_model_params(device: str = "cpu") -> dict[str, Any]:
    """Create default parameters for GazeEstimationModelGeometric initialization.

    Provides a reasonable set of camera and eye parameters for testing.
    Uses identity transforms and centered intrinsics with small camera
    offsets to simulate a binocular eye tracking setup.

    Args:
        device: PyTorch device for tensor allocation (default: "cpu").

    Returns:
        Dictionary containing:
            - camera_left_tf: 4x4 transform for left camera
            - camera_right_tf: 4x4 transform for right camera
            - camera_left_intrinsic: 3x3 intrinsic matrix
            - camera_right_intrinsic: 3x3 intrinsic matrix
            - eye_left_center: 3D position of left eye center
            - eye_right_center: 3D position of right eye center
            - eye_radius: Radius of eye sphere (meters)
    """
    params = {
        "camera_left_tf": torch.eye(4, dtype=torch.float32, device=device),
        "camera_right_tf": torch.eye(4, dtype=torch.float32, device=device),
        "camera_left_intrinsic": torch.tensor(
            [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=torch.float32,
            device=device,
        ),
        "camera_right_intrinsic": torch.tensor(
            [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=torch.float32,
            device=device,
        ),
        "eye_left_center": torch.tensor(
            [-0.03, 0.0, 0.025], dtype=torch.float32, device=device
        ),
        "eye_right_center": torch.tensor(
            [0.03, 0.0, 0.025], dtype=torch.float32, device=device
        ),
        "eye_radius": 0.012,  # float, LearnableMaskedCorrectionParameter handles conversion
    }
    # Apply some default translations to camera TFs
    params["camera_left_tf"][0, 3] = -0.032  # Camera X position
    params["camera_right_tf"][0, 3] = 0.032  # Camera X position
    return params


class TestGazeEstimationModelGeometric:
    """Integration tests for the full geometric gaze estimation model.

    Tests the GazeEstimationModelGeometric class which combines camera
    projection, ray-sphere intersection, and binocular convergence to
    estimate 3D gaze focus points from 2D pupil coordinates.
    """

    @pytest.fixture
    def default_params(self) -> dict[str, Any]:
        """Fixture providing default model parameters."""
        return get_default_model_params()

    @pytest.fixture
    def model(
        self, default_params: dict[str, Any]
    ) -> GazeEstimationModelGeometric:
        """Fixture providing an initialized model with default parameters."""
        return GazeEstimationModelGeometric(**default_params)

    def test_model_initialization(
        self,
        model: GazeEstimationModelGeometric,
        default_params: dict[str, Any],
    ):
        """Test that the model initializes correctly and parameters are set up."""
        assert isinstance(model, GazeEstimationModelGeometric)

        # Check that LearnableMaskedCorrectionParameters are created
        assert isinstance(
            model.camera_left_rotation_tf, LearnableMaskedCorrectionParameter
        )
        assert isinstance(
            model.camera_right_rotation_tf, LearnableMaskedCorrectionParameter
        )
        assert isinstance(
            model.camera_left_position, LearnableMaskedCorrectionParameter
        )
        assert isinstance(
            model.camera_right_position, LearnableMaskedCorrectionParameter
        )
        assert isinstance(
            model.camera_left_intrinsic_inv, LearnableMaskedCorrectionParameter
        )
        assert isinstance(
            model.camera_right_intrinsic_inv,
            LearnableMaskedCorrectionParameter,
        )
        assert isinstance(
            model.eye_left_center, LearnableMaskedCorrectionParameter
        )
        assert isinstance(
            model.eye_right_center, LearnableMaskedCorrectionParameter
        )
        assert isinstance(model.eye_radius, LearnableMaskedCorrectionParameter)

        # Check some initial values stored in the _value attribute of LearnableMaskedCorrectionParameter
        _, expected_left_rot_tf = decompose_tf(
            default_params["camera_left_tf"]
        )
        assert_tensors_equal(
            model.camera_left_rotation_tf._value, expected_left_rot_tf
        )

        expected_left_pos = default_params["camera_left_tf"][:3, 3]
        assert_tensors_equal(
            model.camera_left_position._value, expected_left_pos
        )

        expected_left_intrinsic_inv = torch.linalg.inv(
            default_params["camera_left_intrinsic"]
        )
        assert_tensors_equal(
            model.camera_left_intrinsic_inv._value, expected_left_intrinsic_inv
        )

        assert_tensors_equal(
            model.eye_left_center._value, default_params["eye_left_center"]
        )
        # eye_radius is converted to a tensor by LearnableMaskedCorrectionParameter
        assert_tensors_equal(
            model.eye_radius._value,
            torch.tensor(default_params["eye_radius"], dtype=torch.float32),
        )

    def test_forward_pass_basic_shapes_and_1d_input_handling(
        self, model: GazeEstimationModelGeometric
    ):
        """Test forward pass with single 1D pupil inputs and check output shapes."""
        pupil_left = torch.tensor(
            [320.0, 240.0], dtype=torch.float32
        )  # Center of a typical image
        pupil_right = torch.tensor([320.0, 240.0], dtype=torch.float32)

        focus_point, focus_error = model(
            torch.cat([pupil_left, pupil_right], dim=0)
        )

        assert focus_point.shape == (
            1,
            3,
        ), "Focus point shape mismatch for single input."
        assert focus_error.shape == (1,), (
            "Focus error shape mismatch for single input."
        )

    def test_forward_pass_batch_shapes(
        self, model: GazeEstimationModelGeometric
    ):
        """Test forward pass with batched 2D pupil inputs and check output shapes."""
        pupil_left_batch = torch.tensor(
            [[320.0, 240.0], [300.0, 220.0]], dtype=torch.float32
        )
        pupil_right_batch = torch.tensor(
            [[325.0, 245.0], [310.0, 230.0]], dtype=torch.float32
        )

        focus_point, focus_error = model(
            torch.cat([pupil_left_batch, pupil_right_batch], dim=1)
        )

        assert focus_point.shape == (
            2,
            3,
        ), "Focus point shape mismatch for batch input."
        assert focus_error.shape == (2,), (
            "Focus error shape mismatch for batch input."
        )

    def test_forward_pass_parallel_gaze_error(
        self, default_params: dict[str, Any]
    ):
        """Test scenario where eye gaze rays are parallel, expecting ValueError from nearest_focus."""
        params = default_params.copy()

        eye_x_offset = 0.030  # Eyes are 60mm apart
        eye_z_depth = 0.025  # Eyes are 25mm in front of camera plane (Z=0)

        # Cameras are aligned with eye X positions and point straight (Identity rotation)
        params["camera_left_tf"] = torch.eye(4, dtype=torch.float32)
        params["camera_left_tf"][0, 3] = -eye_x_offset
        params["camera_right_tf"] = torch.eye(4, dtype=torch.float32)
        params["camera_right_tf"][0, 3] = eye_x_offset

        # Simple intrinsics with known principal points (cx, cy)
        fx, fy = 500.0, 500.0
        cx, cy = 160.0, 120.0  # Example principal point
        common_intrinsic = torch.tensor(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=torch.float32,
        )
        params["camera_left_intrinsic"] = common_intrinsic
        params["camera_right_intrinsic"] = common_intrinsic

        params["eye_left_center"] = torch.tensor(
            [-eye_x_offset, 0.0, eye_z_depth], dtype=torch.float32
        )
        params["eye_right_center"] = torch.tensor(
            [eye_x_offset, 0.0, eye_z_depth], dtype=torch.float32
        )
        # Eye radius remains from default_params

        model = GazeEstimationModelGeometric(**params)

        # Pupil coordinates at the principal points should make cameras look straight along their Z-axes.
        # Given the setup, this should result in parallel eye gaze vectors.
        pupil_coords_straight = torch.tensor([cx, cy], dtype=torch.float32)

        with pytest.raises(ValueError, match="Parallel lines detected"):
            model(
                torch.cat(
                    [pupil_coords_straight, pupil_coords_straight], dim=0
                )
            )

    def test_forward_pass_converging_gaze(
        self, default_params: dict[str, Any]
    ):
        """Test a scenario where eye gaze rays should converge, expecting a small focus_error."""
        params = (
            default_params.copy()
        )  # Start with general defaults which are slightly asymmetric

        # Modify for a clear convergence test, similar to parallel but with one pupil offset
        eye_x_offset = 0.030
        eye_z_depth = 0.025

        params["camera_left_tf"] = torch.eye(4, dtype=torch.float32)
        params["camera_left_tf"][
            0, 3
        ] = -eye_x_offset  # Align camera X with eye X
        params["camera_right_tf"] = torch.eye(4, dtype=torch.float32)
        params["camera_right_tf"][0, 3] = (
            eye_x_offset  # Align camera X with eye X
        )

        fx, fy = 500.0, 500.0
        cx, cy = 160.0, 120.0
        common_intrinsic = torch.tensor(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=torch.float32,
        )
        params["camera_left_intrinsic"] = common_intrinsic
        params["camera_right_intrinsic"] = common_intrinsic

        params["eye_left_center"] = torch.tensor(
            [-eye_x_offset, 0.0, eye_z_depth], dtype=torch.float32
        )
        params["eye_right_center"] = torch.tensor(
            [eye_x_offset, 0.0, eye_z_depth], dtype=torch.float32
        )

        model = GazeEstimationModelGeometric(**params)

        pupil_left_straight = torch.tensor([cx, cy], dtype=torch.float32)
        # Offset right pupil's x-coordinate to make it look inwards (towards left)
        pupil_right_converge = torch.tensor(
            [cx - 15, cy], dtype=torch.float32
        )  # 15px offset

        focus_point, focus_error = model(
            torch.cat([pupil_left_straight, pupil_right_converge], dim=0)
        )

        assert focus_point.shape == (1, 3)
        assert focus_error.shape == (1,)

        # For a converging gaze, the error (distance between rays at closest point) should be small.
        # This is a sanity check; an exact value is hard to precompute without extensive geometry.
        # A 15px shift on a 500px focal length is an angle of 15/500 = 0.03 rad ~ 1.7 degrees.
        # This should lead to convergence within a typical viewing distance.
        print(
            f"Converging gaze test: focus_point={focus_point.tolist()}, focus_error={focus_error.item()}"
        )
        assert focus_error.item() < 0.001, (
            "Focus error for converging gaze is unexpectedly high."
        )  # Expect < 1mm error

    def test_backward_pass_and_gradients(
        self, model: GazeEstimationModelGeometric
    ):
        """Test that gradients are computed for learnable parameters' correction terms."""
        # Use pupil inputs that are likely to generate non-zero gradients
        pupil_left = torch.tensor(
            [[320.0 + 5, 240.0 + 5]], dtype=torch.float32
        )  # Slightly off-center
        pupil_right = torch.tensor(
            [[325.0 - 5, 245.0 - 5]], dtype=torch.float32
        )  # Also slightly off-center

        model.train()  # Ensure model is in training mode (though not strictly necessary for this model)

        # Zero gradients using an optimizer is standard practice
        # model.parameters() should yield the ._correction tensors from LearnableMaskedCorrectionParameter
        optimizer = torch.optim.SGD(model.parameters(), lr=0.01)
        optimizer.zero_grad()

        focus_point, focus_error = model(
            torch.cat([pupil_left, pupil_right], dim=1)
        )

        # Define a simple scalar loss
        loss = focus_point.mean() + focus_error.mean()

        # Perform backward pass
        loss.backward()

        found_any_non_zero_grad = False
        num_learnable_model_params = 0

        for param in model.parameters():
            if param.requires_grad:
                num_learnable_model_params += 1
                assert param.grad is not None, (
                    f"A model parameter ({param.shape}) that requires grad has a None gradient."
                )
                if torch.any(param.grad != 0):
                    found_any_non_zero_grad = True

        assert num_learnable_model_params > 0, (
            "No parameters requiring gradients found via model.parameters(). Check LearnableMaskedCorrectionParameter setup."
        )
        assert found_any_non_zero_grad, (
            "All gradients for model parameters requiring grad are zero. "
            "Check model structure, loss function, input data, or if masks/initialization disable all learning."
        )

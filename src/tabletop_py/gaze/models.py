from typing import Any, Literal, Optional

import torch
from torch import nn


def dot(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    """Compute dot product along the last dimension and keep dimensions.

    Args:
        x: First tensor (shape: (..., N))
        y: Second tensor (shape: (..., N))

    Returns:
        Dot product with shape (...,)
    """
    return torch.sum(x * y, dim=-1)


def mv(m: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Multiply a matrix by a vector .

    Args:
        m: Matrix (shape: (..., N, N))
        v: Vector (shape: (..., N))

    Returns:
        Matrix-vector product (shape: (..., N))
    """
    return torch.matmul(m, v.unsqueeze(-1)).squeeze(-1)


def make_homogeneous(x: torch.Tensor) -> torch.Tensor:
    """
    Converts 2D or 3D coordinates to homogeneous coordinates.

    Args:
        x: The 2D or 3D coordinates to convert (shape: (..., 2) or (..., 3))

    Returns:
        The homogeneous coordinates (shape: (..., 3) or (..., 4))
    """
    return torch.cat([x, torch.ones(*x.shape[:-1], 1)], dim=-1)


def decompose_tf(
    tf: torch.Tensor,
    device: torch.device | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """
    Decomposes a transformation matrix into its translation and rotation
    components.

    Args:
        tf: The transformation matrix to decompose (shape: (4, 4))

    Returns:
        translation: The translation matrix (shape: (4, 4))
        rotation: The rotation matrix (shape: (4, 4))
    """
    translation = torch.eye(4, device=device)
    rotation = torch.eye(4, device=device)
    translation[:3, 3] = tf[:3, 3]
    rotation[:3, :3] = tf[:3, :3]
    return translation, rotation


def intersect_ray_sphere(
    ray_origin: torch.Tensor,
    ray_direction: torch.Tensor,
    sphere_center: torch.Tensor,
    sphere_radius: torch.Tensor,
) -> torch.Tensor:
    """
    Calculates the intersection of a ray with a sphere.

    Args:
        ray_origin: Ray origin (shape: (B, 3)).
        ray_direction: Ray direction (shape: (B, 3)).
        sphere_center: Sphere center (shape: (B, 3)).
        sphere_radius: Sphere radius (shape: (B,)).

    Returns:
        The intersection points for each sample in the batch (shape: (B, 3)).

    Raises:
        RuntimeError: If the ray misses the sphere for any sample.
    """
    p, v, c, r = ray_origin, ray_direction, sphere_center, sphere_radius

    v = v / torch.norm(v, dim=-1, keepdim=True)
    d = p - c
    b = dot(v, d)
    c = dot(d, d) - r * r

    # The ray origin should always be outside the sphere
    assert (c > 0).all(), "Ray origin is inside the sphere"
    # The ray should always be pointing towards the sphere
    assert (b < 0).all(), "Ray is pointing away from the sphere"

    discr = b * b - c

    # A negative discriminant corresponds to the ray missing the sphere
    if (discr < 0).any():
        raise RuntimeError("Ray misses the sphere for at least one sample")

    # Ray intersects sphere, compute smallest t value of intersection
    t = -b - torch.sqrt(discr)

    # If t is negative, the ray either started inside the sphere or is
    # pointing away from the sphere, neither of which should happen.
    assert (
        t > 0
    ).all(), (
        "Ray started inside the sphere or is pointing away from the sphere"
    )

    # Return the intersection point
    return p + t.unsqueeze(-1) * v


def nearest_focus(
    p0: torch.Tensor,
    v0: torch.Tensor,
    p1: torch.Tensor,
    v1: torch.Tensor,
    epsilon: float = 1e-8,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Finds the points on two 3D lines that are closest to each other, for a batch of line pairs.
    Also returns the midpoint of the segment connecting these two points (approximate intersection)
    and the distance between the lines.

    Raises ValueError if lines in any batch item are parallel or if any direction vector
    is a zero vector, based on tolerance settings.

    The method is based on finding parameters t and s for lines L1(t) = p0 + t*v0 and L2(s) = p1 + s*v1
    such that the vector connecting L1(t) and L2(s) is perpendicular to v0 and v1.

    Parameters:
    p0 (torch.Tensor): A batch of points on the first lines (shape (B, 3)).
    v0 (torch.Tensor): A batch of direction vectors for the first lines (shape (B, 3)).
    p1 (torch.Tensor): A batch of points on the second lines (shape (B, 3)).
    v1 (torch.Tensor): A batch of direction vectors for the second lines (shape (B, 3)).
    epsilon_sin_sq (float): Tolerance for sin^2(angle between lines) to consider lines parallel.
                              Lines are considered parallel if sin^2(theta) < epsilon_sin_sq.
                              Default is 1e-7 (corresponds to sin(theta) approx 3e-4 rad or 0.017 deg).
                              This also influences the zero vector check: a vector is considered zero if its
                              squared norm is less than epsilon_zero_sq = epsilon_sin_sq * 1e-5.

    Returns:
        Tuple containing:
        - q_mid: Midpoint between nearest points (shape: (B, 3))
        - q0: Nearest point on first line (shape: (B, 3))
        - q1: Nearest point on second line (shape: (B, 3))
        - distance: Distance between the two lines (shape: (B,))
    """
    v0 = v0 / torch.norm(v0, dim=-1, keepdim=True)
    v1 = v1 / torch.norm(v1, dim=-1, keepdim=True)

    v0v1 = dot(v0, v1)
    det = 1 - v0v1 * v0v1

    # Check for parallel lines
    if torch.any(torch.abs(det) < epsilon):
        parallel_indices = torch.where(torch.abs(det) < epsilon)[0].tolist()
        raise ValueError(
            f"Parallel lines detected in batch at indices {parallel_indices}. "
            f"Tolerance for sin_sq_theta (epsilon): {epsilon}"
        )

    dp = p0 - p1
    dpv0 = dot(dp, v0)
    dpv1 = dot(dp, v1)

    t0 = (dpv1 * v0v1 - dpv0) / det
    t1 = (dpv1 - dpv0 * v0v1) / det

    q0 = p0 + t0.unsqueeze(-1) * v0
    q1 = p1 + t1.unsqueeze(-1) * v1

    q_mid = 0.5 * (q0 + q1)

    distance = torch.norm(q0 - q1, dim=-1)

    return q_mid, q0, q1, distance


def nearest_focus_cross(
    p0: torch.Tensor,
    v0: torch.Tensor,
    p1: torch.Tensor,
    v1: torch.Tensor,
    epsilon: float = 1e-8,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Find the nearest point to two 3-dimensional lines.

    The lines are defined by:
        line 0 spanned by p0 + t0 * v0
        line 1 spanned by p1 + t1 * v1

    Args:
        p0: Origin of first line (shape: (B, 3))
        v0: Direction of first line (shape: (B, 3))
        p1: Origin of second line (shape: (B, 3))
        v1: Direction of second line (shape: (B, 3))

    Returns:
        Tuple containing:
        - q_mid: Midpoint between nearest points (shape: (B, 3))
        - q0: Nearest point on first line (shape: (B, 3))
        - q1: Nearest point on second line (shape: (B, 3))
        - distance: Distance between the two lines (shape: (B,))
    """
    # Normalize direction vectors
    v0 = v0 / torch.norm(v0, dim=-1, keepdim=True)
    v1 = v1 / torch.norm(v1, dim=-1, keepdim=True)

    cross = torch.cross(v0, v1, dim=-1)
    if torch.any(torch.norm(cross, dim=-1) < epsilon):
        parallel_indices = torch.where(torch.norm(cross, dim=-1) < epsilon)[
            0
        ].tolist()
        raise ValueError(
            f"Parallel lines detected in batch at indices {parallel_indices}. "
            f"Tolerance for cross product (epsilon): {epsilon}"
        )

    perp0 = torch.cross(v0, cross, dim=-1)
    perp1 = torch.cross(v1, cross, dim=-1)

    dp = p1 - p0

    t0 = dot(dp, perp1) / dot(v0, perp1)
    t1 = -(dot(dp, perp0) / dot(v1, perp0))

    q0 = p0 + t0.unsqueeze(-1) * v0
    q1 = p1 + t1.unsqueeze(-1) * v1

    q_mid = 0.5 * (q0 + q1)
    distance = torch.norm(q0 - q1, dim=-1)

    return q_mid, q0, q1, distance


class LearnableMaskedCorrectionParameter(nn.Module):
    """A masked parameter with a learnable correction

    This class should be used for parameters for which we have a good initial
    guess, but the true value is not known.

    A static parameter is initialized to the initial guess, and the
    tolerance-limited correction is learned as a parameter. The correction is
    only applied to the learnable mask.
    """

    def __init__(
        self,
        initial_value: torch.Tensor | Any,
        learnable_mask: Optional[torch.Tensor | Any] = None,
        max_correction: Optional[float] = None,
        correction_limit_method: Literal["clamp", "tanh"] = "tanh",
        correction_epsilon: float = 1e-3,
    ):
        """
        Args:
            initial_value: The initial value of the parameter.
            learnable_mask: A mask of the same shape as the parameter,
                indicating which elements of the parameter are learnable.
            max_correction: The maximum correction of the parameter, as a
                fraction of the initial value.
            correction_limit_method: The method to limit the correction.
            correction_epsilon: The epsilon to use for the initial correction.
        """
        super().__init__()

        if not isinstance(initial_value, torch.Tensor):
            initial_value = torch.tensor(initial_value)
        self._value = nn.Parameter(initial_value, requires_grad=False)

        if learnable_mask is not None:
            if not isinstance(learnable_mask, torch.Tensor):
                learnable_mask = torch.tensor(learnable_mask, dtype=torch.bool)
            if learnable_mask.shape != self._value.shape:
                raise ValueError(
                    "Learnable mask must have the same shape as the parameter"
                )
            learnable_mask = learnable_mask.bool()
            self._learnable_mask = nn.Parameter(
                learnable_mask,  # type: ignore[arg-type]
                requires_grad=False,
            )
        else:
            self._learnable_mask = 1

        # Initialize the correction to a small random value for numerical stability
        correction_std_init = self._value.abs().min() * correction_epsilon
        self._correction = nn.Parameter(
            torch.randn_like(self._value) * correction_std_init,
            requires_grad=True,
        )

        self._max_correction = max_correction

        if max_correction is None:
            self._correction_fn = self._correction_no_limit
        elif correction_limit_method == "clamp":
            self._correction_fn = self._correction_clamp
        elif correction_limit_method == "tanh":
            self._correction_fn = self._correction_tanh
        else:
            raise ValueError(
                f"Invalid correction limit method: {correction_limit_method}"
            )

    @staticmethod
    def _correction_clamp(
        correction: torch.Tensor, max_correction: float
    ) -> torch.Tensor:
        return torch.clamp(correction, -max_correction, max_correction)

    @staticmethod
    def _correction_tanh(
        correction: torch.Tensor, max_correction: float
    ) -> torch.Tensor:
        return torch.tanh(correction) * max_correction

    @staticmethod
    def _correction_no_limit(
        correction: torch.Tensor, max_correction: None
    ) -> torch.Tensor:
        return correction

    def forward(self) -> torch.Tensor:
        return self._value * (
            1
            + self._learnable_mask
            * self._correction_fn(self._correction, self._max_correction)  # type: ignore[arg-type]
        )


class GazeEstimationModelGeometric(nn.Module):
    """A geometry-based model for estimating 3D gaze from 2D pupil coordinates"""

    # Masks
    _CAMERA_ROTATION_LEARNABLE_MASK = torch.tensor(
        [[1, 1, 1, 0], [1, 1, 1, 0], [1, 1, 1, 0], [0, 0, 0, 0]],
        dtype=torch.bool,
    )
    _CAMERA_INTRINSIC_LEARNABLE_MASK = torch.tensor(
        [[1, 1, 1], [0, 1, 1], [0, 0, 0]],
        dtype=torch.bool,
    )

    def __init__(
        self,
        camera_tf: torch.Tensor | Any,
        camera_intrinsic: torch.Tensor | Any,
        eye_left_center: torch.Tensor | Any,
        eye_right_center: torch.Tensor | Any,
        eye_radius: float | Any,
    ):
        super().__init__()

        # Camera frame transformations
        camera_translation_tf, camera_rotation_tf = decompose_tf(camera_tf)
        self.camera_rotation_tf = LearnableMaskedCorrectionParameter(
            camera_rotation_tf, self._CAMERA_ROTATION_LEARNABLE_MASK
        )
        self.camera_position = LearnableMaskedCorrectionParameter(
            camera_translation_tf[:3, 3]
        )

        # Camera intrinsics
        self.camera_intrinsic_inv = LearnableMaskedCorrectionParameter(
            torch.linalg.inv(camera_intrinsic),
            self._CAMERA_INTRINSIC_LEARNABLE_MASK,
        )

        # Eye position
        self.eye_left_center = LearnableMaskedCorrectionParameter(
            eye_left_center
        )
        self.eye_right_center = LearnableMaskedCorrectionParameter(
            eye_right_center
        )
        self.eye_radius = LearnableMaskedCorrectionParameter(eye_radius)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            x: The input tensor representing the pupil coordinates (shape: (B, 4)).

        Returns:
            Tuple containing:
            - focus_point: Gaze position in world coordinates (shape: (B, 3)).
            - focus_error: Error/distance between the two eye rays (shape: (B,)).
        """
        # Check and flexibly handle dimensionality
        if x.ndim == 1:
            x = x.unsqueeze(0)

        if len(x.shape) != 2 or x.shape[1] != 4:
            raise ValueError("Input must be a 2D tensor of shape (B, 4)")

        # Convert to 2D homogeneous coordinates
        pupil_left_pixels = make_homogeneous(x[:, :2])
        pupil_right_pixels = make_homogeneous(x[:, 2:])

        # Reverse the camera intrinsic projection
        pupil_left_ray_camera = mv(
            self.camera_intrinsic_inv(), pupil_left_pixels
        )
        pupil_right_ray_camera = mv(
            self.camera_intrinsic_inv(), pupil_right_pixels
        )
        # # assert (
        # #     pupil_left_ray_camera.shape
        # #     == pupil_right_ray_camera.shape
        # #     == (B, 3)
        # )

        # Normalize the rays such that the z-coordinate is 1 and convert to 3D
        # homogeneous coordinates
        pupil_left_ray_camera = make_homogeneous(
            pupil_left_ray_camera / pupil_left_ray_camera[:, 2].unsqueeze(-1)
        )
        pupil_right_ray_camera = make_homogeneous(
            pupil_right_ray_camera / pupil_right_ray_camera[:, 2].unsqueeze(-1)
        )
        # assert (
        #     pupil_left_ray_camera.shape
        #     == pupil_right_ray_camera.shape
        #     == (B, 4)
        # )

        # Rotate the rays into the world frame
        pupil_left_ray_world = mv(
            self.camera_rotation_tf(), pupil_left_ray_camera
        )
        pupil_right_ray_world = mv(
            self.camera_rotation_tf(), pupil_right_ray_camera
        )
        # assert (
        #     pupil_left_ray_world.shape == pupil_right_ray_world.shape == (B, 4)
        # )

        # Intersect the rays with the sphere
        pupil_left_pos_world = intersect_ray_sphere(
            ray_origin=self.camera_position(),
            ray_direction=pupil_left_ray_world[:, :3],
            sphere_center=self.eye_left_center(),
            sphere_radius=self.eye_radius(),
        )
        pupil_right_pos_world = intersect_ray_sphere(
            ray_origin=self.camera_position(),
            ray_direction=pupil_right_ray_world[:, :3],
            sphere_center=self.eye_right_center(),
            sphere_radius=self.eye_radius(),
        )
        # assert (
        #     pupil_left_pos_world.shape == pupil_right_pos_world.shape == (B, 3)
        # )

        # Get eye ray direction
        eye_left_ray_world = pupil_left_pos_world - self.eye_left_center()
        eye_right_ray_world = pupil_right_pos_world - self.eye_right_center()
        # assert eye_left_ray_world.shape == eye_right_ray_world.shape == (B, 3)

        # Find the nearest point to the two eye rays
        focus_point, _, _, focus_error = nearest_focus(
            self.eye_left_center(),
            eye_left_ray_world,
            self.eye_right_center(),
            eye_right_ray_world,
        )
        # assert focus_point.shape == (B, 3)
        # assert focus_error.shape == (B,)

        return focus_point, focus_error


class GazeEstimationModelMLP(nn.Module):
    """An MLP model for estimating 3D gaze from 2D pupil coordinates"""

    def __init__(
        self,
        input_size: int,
        output_size: int,
        input_mean: torch.Tensor,
        input_std: torch.Tensor,
        output_mean: torch.Tensor,
        output_std: torch.Tensor,
        hidden_sizes: list[int] = [128, 256, 128],
        dropout_rate: float = 0.2,
        learn_mean_and_std: bool = True,
    ):
        super().__init__()

        self.input_mean = nn.Parameter(
            input_mean.to(torch.float32).unsqueeze(0),
            requires_grad=learn_mean_and_std,
        )
        self.input_std = nn.Parameter(
            input_std.to(torch.float32).unsqueeze(0),
            requires_grad=learn_mean_and_std,
        )
        self.output_mean = nn.Parameter(
            output_mean.to(torch.float32).unsqueeze(0),
            requires_grad=learn_mean_and_std,
        )
        self.output_std = nn.Parameter(
            output_std.to(torch.float32).unsqueeze(0),
            requires_grad=learn_mean_and_std,
        )

        if self.input_mean.ndim != 2 or self.input_mean.shape[1] != input_size:
            raise ValueError(
                f"Input mean length {self.input_mean.shape[1]} does not match input size {input_size}"
            )
        if (
            self.output_mean.ndim != 2
            or self.output_mean.shape[1] != output_size
        ):
            raise ValueError(
                f"Output mean length {self.output_mean.shape[1]} does not match output size {output_size}"
            )
        if self.input_std.ndim != 2 or self.input_std.shape[1] != input_size:
            raise ValueError(
                f"Input std length {self.input_std.shape[1]} does not match input size {input_size}"
            )
        if (
            self.output_std.ndim != 2
            or self.output_std.shape[1] != output_size
        ):
            raise ValueError(
                f"Output std length {self.output_std.shape[1]} does not match output size {output_size}"
            )

        layers = []
        layers.append(nn.Linear(input_size, hidden_sizes[0]))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(dropout_rate))
        for i in range(len(hidden_sizes) - 1):
            layers.append(nn.Linear(hidden_sizes[i], hidden_sizes[i + 1]))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(dropout_rate))
        layers.append(nn.Linear(hidden_sizes[-1], output_size))
        self.layers = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = (x - self.input_mean) / self.input_std
        x = self.layers(x)
        return x * self.output_std + self.output_mean
        # return self.layers(x)

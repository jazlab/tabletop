from typing import Literal, Optional

import torch
from torch import nn


def intersect_ray_sphere(
    o: torch.Tensor,
    u: torch.Tensor,
    c: torch.Tensor,
    r: float,
) -> torch.Tensor | None:
    """
    Calculates the intersection of a ray with a sphere.

    Args:
        o: Ray origin.
        u: Ray direction.
        c: Sphere center.
        r: Sphere radius.

    Returns:
        The intersection point, or None if there is no intersection.
    """
    # Normalize the ray direction
    u = u / torch.norm(u)

    # Compute b and c for the quadratic equation solving for the intersection
    oc = o - c
    b = torch.dot(u, oc)
    c = torch.dot(oc, oc) - r * r

    # The ray origin should always be outside the sphere
    assert c > 0, "Ray origin is inside the sphere"
    # The ray should always be pointing towards the sphere
    assert b < 0, "Ray is pointing away from the sphere"

    discr = b * b - c
    # A negative discriminant corresponds to the ray missing the sphere
    if discr < 0:
        return None  # No intersection

    # Ray intersects sphere, compute smallest t value of intersection
    t = -b - torch.sqrt(discr)

    # If t is negative, the ray either started inside the sphere or is
    # pointing away from the sphere, neither of which should happen.
    assert t > 0, "Ray started inside the sphere"

    # Return the intersection point
    return o + t * u


class LearnableCorrectionParameter(nn.Module):
    """A parameter with a learnable correction

    This class should be used for parameters for which we have a good initial
    guess, but the true value is not known.

    A static parameter is initialized to the initial guess, and the
    tolerance-limited correction is learned as a parameter.
    """

    def __init__(
        self,
        initial_value: torch.Tensor,
        max_correction: Optional[float] = None,
        correction_limit_method: Literal["clamp", "tanh"] = "tanh",
    ):
        """
        Args:
            initial_value: The initial value of the parameter.
            max_correction: The maximum correction of the parameter, as a
                fraction of the initial value.
            correction_limit_method: The method to limit the correction.
        """
        super().__init__()
        self._value = nn.Parameter(initial_value, requires_grad=False)

        # Initialize the correction to a small random value for numerical stability
        correction_std_init = initial_value.abs().min() * 1e-3
        self._correction = nn.Parameter(
            torch.randn_like(initial_value) * correction_std_init,
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

    def __get__(self, obj, objtype=None) -> torch.Tensor:
        return self._value * (
            1 + self._correction_fn(self._correction, self._max_correction)  # type: ignore
        )


class EyeTrackingModel(nn.Module):
    def __init__(
        self,
        camera_left,
        camera_right,
        eye_left_center: torch.Tensor,
        eye_right_center: torch.Tensor,
        eye_radius: float,
        camera_left_intrinsic: torch.Tensor,
        camera_right_intrinsic: torch.Tensor,
        eye_center_max_correction: float = 0.05,
        eye_radius_max_correction: float = 0.05,
        camera_intrinsic_max_correction: float = 0.05,
        camera_extrinsic_max_correction: float = 0.05,
    ):
        super().__init__()

        self.eye_left_center = LearnableCorrectionParameter(
            eye_left_center,
        )
        self.eye_right_center = LearnableCorrectionParameter(
            eye_right_center,
        )
        self.eye_radius = LearnableCorrectionParameter(
            torch.tensor(eye_radius),
        )

        self.camera_left_intrinsic = LearnableCorrectionParameter(
            camera_left_intrinsic
        )
        self.camera_right_intrinsic = LearnableCorrectionParameter(
            camera_right_intrinsic
        )

        # Eye position
        self.eye_tf_mask = nn.Parameter(
            torch.tensor(
                [[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 0, 0, 1]],
                dtype=torch.bool,
            ),
            requires_grad=False,
        )
        self.left_eye_tf = nn.Parameter(torch.randn(4, 4))
        self.right_eye_tf = nn.Parameter(torch.randn(4, 4))

        # Camera intrinsics
        self.intrinsics_mask = nn.Parameter(
            torch.tensor(
                [[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 0, 0, 1]],
                dtype=torch.bool,
            ),
            requires_grad=False,
        )
        self.left_camera_intrinsics = nn.Parameter(torch.randn(4, 4))
        self.right_camera_intrinsics = nn.Parameter(torch.randn(4, 4))

        # Camera extrinsics
        self.extrinsics_mask = nn.Parameter(
            torch.tensor(
                [[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 0, 0, 1]],
                dtype=torch.bool,
            ),
            requires_grad=False,
        )
        self.left_camera_extrinsics = nn.Parameter(torch.randn(4, 4))
        self.right_camera_extrinsics = nn.Parameter(torch.randn(4, 4))

        # self.left_distortion = nn.Parameter(torch.randn(4))
        # self.right_distortion = nn.Parameter(torch.randn(4))

        # self.left_camera_distortion = nn.Parameter(torch.randn(4))
        # self.right_camera_distortion = nn.Parameter(torch.randn(4))

from __future__ import annotations

from dataclasses import dataclass, field
import math
import numpy as np


class InvalidMeasurement(ValueError):
    """Expected rejection of an unsafe/unsupported geometric observation."""


def finite_array(value, shape, name='array'):
    out = np.asarray(value, dtype=np.float64)
    if out.shape != shape or not np.isfinite(out).all():
        raise ValueError(f'{name} must be finite and have shape {shape}')
    return out.copy()


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    K: np.ndarray

    def __post_init__(self):
        k = finite_array(self.K, (3, 3), 'K')
        if (not isinstance(self.width, (int, np.integer)) or isinstance(self.width, bool) or
            not isinstance(self.height, (int, np.integer)) or isinstance(self.height, bool) or
            self.width <= 0 or self.height <= 0 or k[0, 0] <= 0 or k[1, 1] <= 0):
            raise ValueError('invalid camera dimensions/focal length')
        if not np.allclose(k[2], [0, 0, 1]) or not np.isclose(k[1, 0], 0):
            raise ValueError('invalid pinhole intrinsic matrix')
        object.__setattr__(self, 'K', k)

    def rays(self, uv):
        uv = np.asarray(uv, dtype=float)
        return np.column_stack((uv, np.ones(len(uv)))) @ np.linalg.inv(self.K).T


@dataclass(frozen=True)
class Box:
    left: float
    top: float
    right: float
    bottom: float

    def __post_init__(self):
        if not all(math.isfinite(x) for x in self.xyxy):
            raise ValueError('box contains non-finite coordinates')
        if self.right <= self.left or self.bottom <= self.top:
            raise ValueError('box must have positive width and height')

    @property
    def xyxy(self):
        return (self.left, self.top, self.right, self.bottom)

    @property
    def width(self):
        return self.right-self.left

    @property
    def height(self):
        return self.bottom-self.top

    def touches_border(self, camera, margin=1.):
        return (self.left <= margin or self.top <= margin or
                self.right >= camera.width-margin or self.bottom >= camera.height-margin)

    def clipped(self, camera):
        a, b = max(0., self.left), max(0., self.top)
        c, d = min(float(camera.width), self.right), min(float(camera.height), self.bottom)
        return None if c <= a or d <= b else Box(a, b, c, d)

    def iou(self, other):
        area = max(0., min(self.right, other.right)-max(self.left, other.left))*max(
            0., min(self.bottom, other.bottom)-max(self.top, other.top))
        return area/(self.width*self.height + other.width*other.height-area)


@dataclass(frozen=True)
class Detection:
    box: Box
    label: str
    confidence: float

    def __post_init__(self):
        if self.label not in ('crater', 'crater_partial'):
            raise ValueError('unsupported crater label')
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('invalid detection confidence')


@dataclass
class Plane:
    normal: np.ndarray
    offset: float
    rms_m: float = 0.
    inlier_ratio: float = 1.
    support_m: float = 1.
    count: int = 0

    def __post_init__(self):
        n = finite_array(self.normal, (3,), 'normal')
        norm = np.linalg.norm(n)
        if norm < 1e-9 or not math.isfinite(self.offset):
            raise ValueError('invalid plane')
        self.normal, self.offset = n/norm, float(self.offset)/norm


@dataclass
class Circle:
    center: np.ndarray
    normal: np.ndarray
    radius: float
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    diameter_variance: float = 0.
    residual_m: float = 0.
    reprojection_px: float = 0.

    def __post_init__(self):
        self.center = finite_array(self.center, (3,), 'center')
        self.normal = finite_array(self.normal, (3,), 'normal')
        self.covariance = finite_array(self.covariance, (3, 3), 'covariance')
        if abs(np.linalg.norm(self.normal)-1) > 1e-4:
            raise ValueError('circle normal must have unit length')
        if (not np.allclose(self.covariance, self.covariance.T, atol=1e-9) or
            np.linalg.eigvalsh(self.covariance).min() < -1e-9):
            raise ValueError('circle covariance must be symmetric positive semidefinite')
        if (not all(math.isfinite(v) for v in (self.radius, self.diameter_variance,
                                             self.residual_m, self.reprojection_px)) or
            self.radius <= 0 or min(self.diameter_variance, self.residual_m, self.reprojection_px) < 0):
            raise ValueError('invalid circle dimensions/uncertainty')

    @property
    def diameter(self):
        return 2*self.radius


def validate_transform(transform):
    t = finite_array(transform, (4, 4), 'transform')
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError('invalid homogeneous transform')
    r = t[:3, :3]
    if not np.allclose(r.T@r, np.eye(3), atol=1e-5) or np.linalg.det(r) < .999:
        raise ValueError('transform rotation must be orthonormal and right-handed')
    return t


def transform_circle(circle, transform, extra_covariance=None):
    t = validate_transform(transform)
    r = t[:3, :3]
    covariance = r@circle.covariance@r.T
    if extra_covariance is not None:
        covariance += finite_array(extra_covariance, (3, 3), 'extra_covariance')
    return Circle(r@circle.center+t[:3, 3], r@circle.normal, circle.radius,
                  covariance, circle.diameter_variance, circle.residual_m, circle.reprojection_px)

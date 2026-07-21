"""
pyvn100.types — Shared data structures.

Field-for-field match with the C-side `vn100_data_t` (see vn100_types.h — M7).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Vn100Data:
    """A single VN-100 measurement (decoded from a $VNYMR message or a binary packet)."""

    # Euler angles [degrees]
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    # Magnetic field [Gauss]
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0

    # Acceleration [m/s^2]
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # Angular rate [rad/s]
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    # Meta — filled in on the host side (optional)
    timestamp: float | None = None   # host receive time [s]

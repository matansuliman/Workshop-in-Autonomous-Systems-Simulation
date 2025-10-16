# core/LowLevel/SimulationDecoupling/backend.py
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Iterable, Optional, Dict
import numpy as np


class PhysicsBackend(ABC):
    """Abstract interface for a physics backend (e.g., MuJoCo, PyBullet)."""

    @abstractmethod
    def step_once(self) -> None:
        """Advance the simulation by one internal timestep."""
        ...

    @property
    @abstractmethod
    def dt(self) -> float:
        """Return simulation timestep."""
        ...

    @abstractmethod
    def get_time(self) -> float:
        """Return current simulation time."""
        ...

    # External forces
    @abstractmethod
    def apply_force(self, body: str | int, force_world: Iterable[float], torque_world: Optional[Iterable[float]] = None,
                    *, mode: str = "add", ) -> None:
        ...

    @abstractmethod
    def clear_external_forces(self) -> None:
        """Clear all applied forces."""
        ...

    # World / physics options
    @abstractmethod
    def set_gravity(self, g_world: Iterable[float]) -> None: ...

    @abstractmethod
    def set_air_density(self, rho: float) -> None: ...

    @abstractmethod
    def set_viscosity(self, mu: float) -> None: ...

    @abstractmethod
    def set_timestep(self, dt: float) -> None: ...

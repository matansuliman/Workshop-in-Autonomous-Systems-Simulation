# core/LowLevel/SimulationDecoupling/mujoco_backend.py
from __future__ import annotations
from typing import Iterable, Optional
import numpy as np
import mujoco

from ...LowLevel.SimulationDecoupling.backend import PhysicsBackend


class MujocoBackend(PhysicsBackend):
    """Concrete implementation of PhysicsBackend using MuJoCo."""

    def __init__(self, xml_path: str):
        self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._data = mujoco.MjData(self._model)

    # ---- core timing ----
    @property
    def dt(self) -> float:
        return self._model.opt.timestep

    def get_time(self) -> float:
        return self._data.time

    # ---- stepping ----
    def step_once(self) -> None:
        mujoco.mj_step(self._model, self._data)

    # ---- external forces ----
    def apply_force(
        self,
        body: str | int,
        force_world: Iterable[float],
        torque_world: Optional[Iterable[float]] = None,
        *,
        mode: str = "add",
    ) -> None:
        f = np.asarray(force_world, dtype=float).reshape(3)
        t = np.asarray(torque_world or (0.0, 0.0, 0.0), dtype=float).reshape(3)
        bid = self._model.body(body).id if isinstance(body, str) else int(body)

        if mode == "set":
            self._data.xfrc_applied[bid, :3] = f
            self._data.xfrc_applied[bid, 3:] = t
        elif mode == "add":
            self._data.xfrc_applied[bid, :3] += f
            self._data.xfrc_applied[bid, 3:] += t
        else:
            raise ValueError("mode must be 'set' or 'add'")

    def clear_external_forces(self) -> None:
        self._data.xfrc_applied[:, :] = 0.0

    # ---- world parameters ----
    def set_gravity(self, g_world: Iterable[float]) -> None:
        self._model.opt.gravity[:] = np.asarray(g_world, dtype=float).reshape(3)

    def set_air_density(self, rho: float) -> None:
        self._model.opt.density = float(rho)

    def set_viscosity(self, mu: float) -> None:
        self._model.opt.viscosity = float(mu)

    def set_timestep(self, dt: float) -> None:
        self._model.opt.timestep = float(dt)

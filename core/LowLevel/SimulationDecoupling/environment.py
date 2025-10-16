# core/LowLevel/SimulationDecoupling/environment.py
from __future__ import annotations
from dataclasses import dataclass, field

from typing import Dict, Iterable, Optional, Union
import mujoco
import mujoco.viewer
import glfw
import numpy as np

from ...LowLevel.SimulationDecoupling.mujoco_backend import MujocoBackend


@dataclass
class WindConfig:
    enabled: bool = False
    velocity_world: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # m/s, world frame
    air_density: float = 1.225  # kg/m^3
    # Per-body drag coefficient * reference area (Cd * A) [m^2]
    # e.g., {"x2": 0.05, "platform": 0.2}
    cda_by_body: Dict[str, float] = field(default_factory=dict)


class ENV:
    def __init__(self, path_to_xml):

        self._backend = MujocoBackend(path_to_xml)

        # will be deprecated
        self._model = self._backend.model
        self._data = self._backend.data
        self._dt = self._backend.dt


        # --- Extras state ---
        self._wind = WindConfig()

        # Fast name->id caches (created lazily)
        self._body_id_cache: Dict[str, int] = {}
        self._joint_id_cache: Dict[str, int] = {}
        self._act_id_cache: Dict[str, int] = {}
        self._sensor_id_cache: Dict[str, int] = {}

    def launch_viewer(self):
        return self._backend.launch_viewer()

    # -------------------- Core properties --------------------
    @property
    def model(self) -> mujoco.MjModel:
        return self._backend.model

    @property
    def data(self) -> mujoco.MjData:
        return self._backend.data

    @property
    def dt(self) -> float:
        return self._backend.dt

    def get_time(self) -> float:
        return self._backend.get_time()

    # -------------------- Simulation stepping --------------------
    def step(self, n_substeps: int = 1):
        """
        Advance simulation by n_substeps * model.opt.timestep while applying configured effects.
        Replace direct calls to mujoco.mj_step(...) with env.step() to enable wind/drag, etc.
        """
        for _ in range(n_substeps):
            # Clear any previously applied external forces that *we* manage
            self.clear_external_forces()

            # Inject wind/drag if enabled
            if self._wind.enabled:
                self._apply_wind_drag()

            self._backend.step_once()

    # -------------------- Wind / aerodynamic drag --------------------
    def enable_wind(self, enabled: bool = True):
        self._wind.enabled = enabled

    def set_wind(self, velocity_world: Union[Iterable[float], np.ndarray], air_density: Optional[float] = None):
        self._wind.velocity_world = np.asarray(velocity_world, dtype=float).reshape(3)
        if air_density is not None:
            self._wind.air_density = float(air_density)

    def get_wind_vel(self):
        return self._wind.velocity_world

    def set_body_cda(self, body: Union[str, int], cda: float):
        """Set Cd*A (m^2) used for quadratic drag on a body."""
        name = self.body_name(body)
        self._wind.cda_by_body[name] = float(cda)

    def clear_cda(self, body: Optional[Union[str, int]] = None):
        if body is None:
            self._wind.cda_by_body.clear()
        else:
            name = self.body_name(body)
            self._wind.cda_by_body.pop(name, None)

    def _apply_wind_drag(self):
        """
        Apply simple quadratic drag to configured bodies:
            F = 0.5 * rho * CdA * |v_rel| * v_rel  (opposes body motion relative to wind)
        Forces are applied at body COM in world frame (xfrc_applied).
        """
        rho = self._wind.air_density
        v_w = self._wind.velocity_world

        if not self._wind.cda_by_body:
            return  # nothing to do

        for body_name, cda in self._wind.cda_by_body.items():
            # get world-frame COM velocity from backend
            v_lin_world = self._backend.get_body_linvel(body_name)

            # relative wind: air minus body velocity (world frame)
            v_rel = v_w - v_lin_world
            speed = np.linalg.norm(v_rel)
            if speed <= 1e-9 or cda <= 0.0 or rho <= 0.0:
                continue

            # Quadratic drag (world frame)
            f = 0.5 * rho * cda * speed * v_rel

            # Apply at COM through backend
            self._backend.apply_force(body_name, f)

    # -------------------- External forces helpers --------------------
    def apply_force(self, body: str | int, force_world: Iterable[float], torque_world: Optional[Iterable[float]] = None,
                    *, mode: str = "add", ) -> None:
        """
        Apply an external spatial force (world frame) on a body COM for the current step.
        If mode == 'set' it overwrites; if 'add' it accumulates.
        """
        self._backend.apply_force(body, force_world, torque_world, mode=mode)

    def clear_external_forces(self):
        """Zero out all xfrc_applied for this step (safe to call every step)."""
        self._backend.clear_external_forces()

    # -------------------- World/options setters --------------------
    def set_gravity(self, g_world: Union[Iterable[float], np.ndarray]):
        self._backend.set_gravity(g_world)

    def set_air_density(self, rho: float):
        self._backend.set_air_density(rho)

    def set_viscosity(self, mu: float):
        self._backend.set_viscosity(mu)

    def set_timestep(self, dt: float):
        self._backend.set_timestep(dt)

    # -------------------- Reset helpers --------------------
    def reset(self, qpos: Optional[Iterable[float]] = None, qvel: Optional[Iterable[float]] = None) -> None:
        """mj_resetData + optional state overwrite."""
        self._backend.reset(qpos, qvel)

    # -------------------- Body / joint identifiers --------------------
    def body_id(self, body: Union[str, int]) -> int:
        return self._backend.body_id(body)

    def body_name(self, body: Union[str, int]) -> str:
        return self._backend.body_name(body)

    def joint_id(self, joint: Union[str, int]) -> int:
        return self._backend.joint_id(joint)

    def actuator_id(self, actuator: Union[str, int]) -> int:
        return self._backend.actuator_id(actuator)

    def sensor_id(self, sensor: Union[str, int]) -> int:
        return self._backend.sensor_id(sensor)

    def sensor_name(self, sensor: Union[str, int]) -> str:
        return self._backend.sensor_name(sensor)

    # ---- Poses for free bodies and joints ----
    def set_free_body_pose(self, body: Union[str, int],
                           pos_world: Optional[Iterable[float]] = None, quat_wxyz: Optional[Iterable[float]] = None):
        self._backend.set_free_body_pose(body, pos_world, quat_wxyz)

    def set_free_body_velocity(self, body: Union[str, int],
                               linvel_world: Optional[Iterable[float]] = None, angvel_world: Optional[Iterable[float]] = None):
        self._backend.set_free_body_velocity(body, linvel_world, angvel_world)

    def set_joint_qpos(self, joint: Union[str, int], value: float):
        jid = self.joint_id(joint)
        adr = self._model.jnt_qposadr[jid]
        # Hinge, slide -> scalar
        self._data.qpos[adr] = float(value)

    def set_joint_qvel(self, joint: Union[str, int], value: float):
        jid = self.joint_id(joint)
        adr = self._model.jnt_dofadr[jid]
        self._data.qvel[adr] = float(value)

    # -------------------- Actuators / controls --------------------
    def set_ctrl(self, actuator: Union[str, int], value: float):
        aid = self.actuator_id(actuator)
        self._data.ctrl[aid] = float(value)

    def set_ctrls(self, values_by_name: Dict[str, float]):
        for name, val in values_by_name.items():
            self.set_ctrl(name, val)

    def actuators_for_body(self, body: int | str):
        """
        Return actuator ids and names that act on the given body.
        Works for JOINT-driven and SITE-driven actuators.
        """
        if isinstance(body, str):
            body_id = self.body_id(body)
        else:
            body_id = int(body)

        mdl = self._model
        ids, names = [], []

        for aid in range(mdl.nu):
            trntype = int(mdl.actuator_trntype[aid])
            idx0 = int(mdl.actuator_trnid[aid, 0])

            # JOINT-driven
            if trntype == mujoco.mjtTrn.mjTRN_JOINT:
                if 0 <= idx0 < mdl.njnt and mdl.jnt_bodyid[idx0] == body_id:
                    ids.append(aid)
                    names.append(mdl.actuator(aid).name)
                continue

            # SITE-driven (e.g. quadrotor motors)
            if trntype == mujoco.mjtTrn.mjTRN_SITE:
                if 0 <= idx0 < mdl.nsite and mdl.site_bodyid[idx0] == body_id:
                    ids.append(aid)
                    names.append(mdl.actuator(aid).name)
                continue

        return ids, names

    # -------------------- Convenience: quick “hover” gravity tweak --------------------
    def set_gravity_scale(self, scale: float):
        """Scale current gravity vector magnitude by 'scale' (sign preserved)."""
        g = self._model.opt.gravity.copy()
        mag = np.linalg.norm(g)
        if mag > 0:
            self._backend.set_gravity(g * scale)

    # -------------------- Debug helpers --------------------
    def world_linvel_of_body(self, body: Union[str, int]) -> np.ndarray:
        return self._backend.get_body_linvel(body)

    def world_pos_of_body(self, body: Union[str, int]) -> np.ndarray:
        return self._backend.get_body_pos(body)